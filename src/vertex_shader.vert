#version 460 core

struct MeshData {
   vec4 position;            // Offset= 0, size=16 bytes.
   vec4 velocity;            // Offset=16, size=16 bytes.
   vec4 orientation;         // Offset=32, size=16 bytes. Quaternion
   vec4 angVel;              // Offset=48, size=16 bytes. Unit axis (xyz)
   vec4 centerOfRotation;    // Offset=64, size=16 bytes.
   uint time;                // Offset=80, size= 4 bytes.
   int colorTextureUnit;     // Offset=84, size= 4 bytes. (-1 means no textures)
   int normalTextureUnit;    // Offset=88, size= 4 bytes. (-1 means no textures)
   uint padding2;            // Offset=92, size= 4 bytes. (padding)
}; // Make sure to pad so size is divisible by 16 because you have a vec4.

layout(std430, binding = 0) buffer MeshDataBuffer {
   MeshData meshDataBuffer[];
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 tangent;
layout (location = 3) in vec2 uv;
layout (location = 4) in float occlusionFactor;
layout (location = 5) in uint meshIndex;
layout (location = 6) in uint triangleIndex;

uniform uint u_frame;
uniform uint u_time;
uniform float u_timeRemainder;

uniform mat4 view;
uniform mat4 projection;

out vec3 vert_normal;
out mat3 vert_TBN;
out vec3 vert_pos;
out vec2 vert_uv;
flat out int vert_colorTextureUnit;
flat out int vert_normalTextureUnit;
out float vert_occlusionFactor;

mat3 fromQuaternion(vec4 quaternion) {
    float qw = quaternion.w;
    float qx = quaternion.x;
    float qy = quaternion.y;
    float qz = quaternion.z;

    // Calculate coefficients
    float xx = qx * qx;
    float yy = qy * qy;
    float zz = qz * qz;
    float xy = qx * qy;
    float xz = qx * qz;
    float yz = qy * qz;
    float wx = qw * qx;
    float wy = qw * qy;
    float wz = qw * qz;

    return mat3(
       1.0 - 2.0 * (yy + zz),       2.0 * (xy + wz),       2.0 * (xz - wy),
             2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz),       2.0 * (yz + wx),
             2.0 * (xz + wy),       2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)
    );
}

mat3 rotationMatrix(float angle, vec3 unitAxis) {
    float cosAngle = cos(angle);
    float sinAngle = sin(angle);
    float ux = unitAxis.x;
    float uy = unitAxis.y;
    float uz = unitAxis.z;

    return mat3(
        cosAngle + ux * ux * (1.0 - cosAngle),
        uy * ux * (1.0 - cosAngle) + uz * sinAngle,
        uz * ux * (1.0 - cosAngle) - uy * sinAngle,

        ux * uy * (1.0 - cosAngle) - uz * sinAngle,
        cosAngle + uy * uy * (1.0 - cosAngle),
        uz * uy * (1.0 - cosAngle) + ux * sinAngle,

        ux * uz * (1.0 - cosAngle) + uy * sinAngle,
        uy * uz * (1.0 - cosAngle) - ux * sinAngle,
        cosAngle + uz * uz * (1.0 - cosAngle)
    );
}

void main() {
   vert_normal = normalize(normal);
   vert_occlusionFactor = occlusionFactor;

   MeshData meshData = meshDataBuffer[meshIndex];
   vert_uv = uv;
   vert_colorTextureUnit = meshData.colorTextureUnit;
   vert_normalTextureUnit = meshData.normalTextureUnit;

   uint deltaTime = u_time - meshData.time;
   float deltaTimeFloat = float(deltaTime) + u_timeRemainder;

   vec3 meshPosition = meshData.position.xyz + meshData.velocity.xyz * deltaTimeFloat;
   
   mat3 orientation = fromQuaternion(meshData.orientation);
   orientation = rotationMatrix(
      meshData.angVel.w * deltaTimeFloat * 1., meshData.angVel.xyz
   ) * orientation;
   
   vec3 rotatedPosition = orientation * (
      position - meshData.centerOfRotation.xyz
   ) + meshData.centerOfRotation.xyz;
   
   // Transform normal and tangent
   vec3 N = normalize(orientation * normal);
   vec3 T = normalize(orientation * tangent);
   
   // Re-orthogonalize T with respect to N using Gram-Schmidt process
   T = normalize(T - dot(T, N) * N);
   
   // Calculate bitangent B
   vec3 B = cross(N, T);
   
   // Build TBN matrix for normal mapping
   vert_TBN = mat3(T, B, N);
   vert_normal = N;

   vec4 worldPos = vec4(meshPosition + rotatedPosition, 1.0);
   vert_pos = worldPos.xyz;
   gl_Position = projection * view * worldPos;
}
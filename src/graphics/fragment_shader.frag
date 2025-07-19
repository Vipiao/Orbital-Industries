#version 460 core

out vec4 FragColor;

uniform vec3 u_cameraPositionHigh;
uniform vec3 u_cameraPositionLow;
uniform sampler2D u_textures[16];
uniform uint u_time;
uniform float u_timeRemainder;
uniform vec3 u_lightPos;

in vec3 vert_normal;
in mat3 vert_TBN;
in vec3 vert_pos;
in vec2 vert_uv;
flat in int vert_colorTextureUnit;
flat in int vert_normalTextureUnit;
in float vert_occlusionFactor;

void main() {
   // Get normal from normal map or use vertex normal
   vec3 normal;
   if (vert_normalTextureUnit == -1) {
      normal = normalize(vert_normal);
   } else {
      // Sample normal map and transform to world space using TBN matrix
      vec3 normalMap = texture(u_textures[vert_normalTextureUnit], vert_uv).rgb;
      normalMap = normalMap * 2.0 - 1.0;  // Convert from [0,1] to [-1,1] range
      normal = normalize(vert_TBN * normalMap);  // Transform from tangent to world space
   }

   // Get object color from texture or use white as default
   vec3 objectColor;
   if (vert_colorTextureUnit >= 0) {
      objectColor = texture(u_textures[vert_colorTextureUnit], vert_uv).rgb;
   } else {
      objectColor = vec3(1.0, 1.0, 1.0);
   }

   // Calculate light and view directions (all in L-space now)
   vec3 lightVec = u_lightPos - vert_pos;
   vec3 lightDir = normalize(lightVec);
   vec3 viewDir = normalize(-vert_pos); // Camera is at origin in L-space
   float sqrDist = dot(lightVec, lightVec);
   float attenuation = 32./sqrDist;// + 4./length(lightVec);
   attenuation = 1. - 1./(1+attenuation); // Soft max 1.
   attenuation = 1.;
   
   // Phong lighting model components
   
   // 1. Ambient light - base illumination
   float ambientStrength = 0.2;
   vec3 ambient = ambientStrength * objectColor;
   
   // 2. Diffuse light - varies with surface orientation to light
   float diff = max(dot(normal, lightDir), 0.0);
   vec3 diffuse = diff * objectColor;
   
   // 3. Specular light - reflective highlights
   float specularStrength = 0.5;
   vec3 reflectDir = reflect(-lightDir, normal);
   float spec = pow(max(min(dot(viewDir, reflectDir) + 0.001, 1.), 0.0), 128.0);
   vec3 specular = specularStrength * spec * vec3(1.0);
   
   // Apply occlusion factor to all lighting components
   // 0.0 = fully occluded, 1.0 = no occlusion
   ambient *= vert_occlusionFactor;
   diffuse *= vert_occlusionFactor;
   specular *= vert_occlusionFactor;
   
   // Combine all lighting components
   vec3 result = ambient + (diffuse + specular) * attenuation;
   
   FragColor = vec4(result, 1.0);
}
#version 460 core

out vec4 FragColor;

uniform sampler2D gAlbedo;
uniform sampler2D gNormal;
uniform sampler2D gPosition;
uniform sampler2D gMaterial;
uniform vec3 u_lightPos;
uniform mat4 u_projection;
uniform bool u_ssaoEnabled;
uniform float u_timeRemainder;
uniform float u_ssaoRadius;
uniform float u_ssaoBias;
uniform vec3 u_ssaoSamples[64];

in vec2 texCoord;

vec3 debugColor = vec3(0.);

float calculateSSAO(vec3 fragPos, vec3 normal) {
   if (!u_ssaoEnabled) {
      return 1.0;
   }
   
   // Generate random tangent vector that rotates each frame
   vec2 timeOffset = vec2(u_timeRemainder * 0.1, u_timeRemainder * 0.13);
   vec3 randomVec = vec3(
      fract(sin(dot(texCoord + timeOffset, vec2(12.9898, 78.233))) * 43758.5453),
      fract(sin(dot(texCoord + timeOffset + vec2(0.1, 0.1), vec2(12.9898, 78.233))) * 43758.5453),
      0.0
   );
   
   // Create TBN matrix
   vec3 tangent = normalize(randomVec - normal * dot(randomVec, normal));
   vec3 bitangent = cross(normal, tangent);
   mat3 TBN = mat3(tangent, bitangent, normal);
   
   float occlusion = 0.0;
   int sampleCount = 60; // Match the uniform array size
   
   //debugColor.x = 0.;
   for (int i = 0; i < sampleCount; ++i) {
      // Get sample position in world space
      vec3 samplePos = TBN * u_ssaoSamples[i];
      samplePos = fragPos + samplePos * u_ssaoRadius;
      
      // Project to screen space
      vec4 offset = u_projection * vec4(samplePos, 1.0);
      offset.xyz /= offset.w;
      offset.xyz = offset.xyz * 0.5 + 0.5;
      
      // Check if sample is within screen bounds
      if (offset.x < 0.0 || offset.x > 1.0 || offset.y < 0.0 || offset.y > 1.0) {
         continue;
      }
      
      // Sample depth at this position
      float sampleDepth = texture(gPosition, offset.xy).z;
      
      // Range check to reduce artifacts
      float haloFactor = 2.;
      float rangeCheck = smoothstep(0.0, 1.0, haloFactor * u_ssaoRadius / abs(fragPos.z - sampleDepth));
      
      // Compare depths
      occlusion += (sampleDepth >= samplePos.z + u_ssaoBias ? 1.0 : 0.0) * rangeCheck;
      //debugColor.x *= rangeCheck;
      //debugColor.x = max(debugColor.x, abs(fragPos.z - sampleDepth));
   }
   //debugColor.x = occlusion / float(sampleCount);
   
   occlusion = 1.0 - (occlusion / float(sampleCount));
   
   return occlusion;
}

void main() {
   // Sample G-buffer
   vec4 albedoSample = texture(gAlbedo, texCoord);
   vec4 normalSample = texture(gNormal, texCoord);
   vec4 positionSample = texture(gPosition, texCoord);
   vec4 materialSample = texture(gMaterial, texCoord);
   
   vec3 albedo = albedoSample.rgb;
   float metallic = albedoSample.a;
   vec3 normal = normalSample.rgb * 2.0 - 1.0;  // Decode normal from [0,1] to [-1,1]
   float roughness = normalSample.a;
   vec3 fragPos = positionSample.rgb;
   //float ao = positionSample.a;
   float emissiveStrength = materialSample.r;
   float geometryFlag = materialSample.g;
   float alpha = materialSample.a;

    
   // Detect background pixels and discard them to preserve sky background
   if (geometryFlag < 0.5) {
       discard;
   }
   
   // Early exit for full emissive materials
   if (emissiveStrength >= 0.999) {
      FragColor = vec4(albedo, alpha);
      return;
   }

   // Calculate SSAO
   float ssaoFactor = calculateSSAO(fragPos, normal);
   ssaoFactor = pow(ssaoFactor, 1.5); // TEST
   
   // Calculate light and view directions (all in L-space)
   vec3 lightVec = u_lightPos - fragPos;
   vec3 lightDir = normalize(lightVec);
   vec3 viewDir = normalize(-fragPos); // Camera is at origin in view space
   float sqrDist = dot(lightVec, lightVec);
   float attenuation = 32.0/sqrDist;
   //attenuation = 1.0 - 1.0/(1.0+attenuation); // Soft max 1.
   attenuation = 1.0;
   
   // Phong lighting model
   float ambientStrength = 0.3;
   vec3 ambient = ambientStrength * albedo * ssaoFactor;
   
   float diff = max(dot(normal, lightDir), 0.0);
   //diff += max(dot(normal, -lightDir), 0.0) * 0.1;

   vec3 diffuse = diff * albedo;// * mix(1.0, ssaoFactor, 0.2);
   
   float specularStrength = 0.5;
   vec3 reflectDir = reflect(-lightDir, normal);
   float spec = pow(max(min(dot(viewDir, reflectDir) + 0.001, 1.0), 0.0), 128.0);
   vec3 specular = specularStrength * spec * vec3(1.0);
   
   //float ff = pow(smoothstep(0., 0.5, ssaoFactor), 2.); // TEST
   //float ff = mix(1.0, ssaoFactor, 1.-pow(ssaoFactor, 3.)); // TEST
   
   vec3 result = (ambient + (diffuse + specular) * attenuation) * mix(1.0, ssaoFactor, 0.2);
   result = mix(result, albedo, emissiveStrength);
   
   FragColor = vec4(result, 1.);
   //debugColor.yz = result.yz;
   //FragColor = vec4(debugColor, 1.);
}
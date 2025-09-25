#version 460 core

out vec4 FragColor;

uniform sampler2D gAlbedo;
uniform sampler2D gNormal;
uniform sampler2D gMaterial;
uniform sampler2D gDepth;
uniform sampler2D u_shadowMap;
uniform mat4 u_lightSpaceMatrix;
uniform bool u_shadowsEnabled;
uniform vec3 u_lightDir;
uniform mat4 u_projection;
uniform mat4 u_inverseProjection;
uniform vec2 u_screenSize;
uniform bool u_ssaoEnabled;
uniform float u_timeRemainder;
uniform float u_ssaoRadius;
uniform float u_ssaoBias;
uniform vec3 u_ssaoSamples[32];

in vec2 texCoord;

vec3 debugColor = vec3(0.);

vec3 reconstructPosition(vec2 screenCoord, float depth) {
   // Convert screen coordinates to NDC
   vec2 ndc = screenCoord * 2.0 - 1.0;
   
   // Create clip space coordinates
   vec4 clipSpace = vec4(ndc, depth * 2.0 - 1.0, 1.0);
   
   // Transform to view space
   vec4 viewSpace = u_inverseProjection * clipSpace;
   return viewSpace.xyz / viewSpace.w;
}

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
   int sampleCount = 32; // Match the uniform array size
   
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
      vec3 reconstructedPos = reconstructPosition(offset.xy, texture(gDepth, offset.xy).r);
      float sampleDepth = reconstructedPos.z;
      
      // Range check to reduce artifacts
      float haloFactor = 1.;
      float rangeCheck = smoothstep(0.0, 1.0, haloFactor * u_ssaoRadius / abs(fragPos.z - sampleDepth));
      
      // Compare depths
      //occlusion += (sampleDepth >= samplePos.z + u_ssaoBias ? 1.0 : 0.0) * rangeCheck;
      float depthDiff = sampleDepth - samplePos.z;
      float occlusionContribution = smoothstep(-u_ssaoBias, u_ssaoBias, depthDiff);
      occlusion += occlusionContribution * rangeCheck;
      //debugColor.x *= rangeCheck;
      //debugColor.x = max(debugColor.x, abs(fragPos.z - sampleDepth));
   }
   //debugColor.x = occlusion / float(sampleCount);
   
   occlusion = 1.0 - (occlusion / float(sampleCount));
   
   return occlusion;
}

// Simple hash function for per-pixel randomness
float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

vec2 hash2(vec2 p) {
    return fract(sin(vec2(dot(p, vec2(127.1, 211.7)), dot(p, vec2(169.5, 183.3)))) * 43758.5453123);
}

float calculateShadow(
   vec3 fragPos, vec3 normal, vec3 lightDir, float bias
) {
    if (!u_shadowsEnabled) {
        return 1.0; // No shadow
    }
    
    // Transform fragment position to light space
    vec4 fragPosLightSpace = u_lightSpaceMatrix * vec4(fragPos, 1.0);
    
    // Perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    
    // Transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    
    // Check if fragment is outside shadow map bounds
    if (projCoords.x < 0.0 || projCoords.x > 1.0 || projCoords.y < 0.0 || projCoords.y > 1.0) {
        return 1.0; // Outside shadow map, assume no shadow
    }
    
    // Get current fragment depth in light space
    float currentDepth = projCoords.z;
    
    // Calculate bias to prevent shadow acne
    //float bias = max(0.005 * (1.0 - dot(normal, lightDir)), 0.001);
    //float bias = 0.;

    // Temporal jittering: add per-pixel random offsets that vary over time
    vec2 screenPos = gl_FragCoord.xy;
    vec2 timeOffset = vec2(u_timeRemainder * 0.3, u_timeRemainder * 0.7); // Different time scales for X/Y
    vec2 jitterSeed = screenPos + timeOffset;
    vec2 jitter = (hash2(jitterSeed) - 0.5) * 2.0; // [-1, 1] range
    vec2 texelSize = 1.0 / textureSize(u_shadowMap, 0);
    vec2 jitteredOffset = jitter * texelSize; // Scale to texel size

    // PCF (Percentage Closer Filtering) for soft shadows
    float shadow = 0.0;
    for(int x = -1; x <= 1; ++x) {
    //for(int x = 0; x < 1; ++x) {
        for(int y = -1; y <= 1; ++y) {
        //for(int y = 0; y < 1; ++y) {
            // Apply both PCF offset and temporal jitter
            vec2 sampleOffset = vec2(x, y) * texelSize + jitteredOffset;
            float pcfDepth = texture(u_shadowMap, projCoords.xy + sampleOffset).r;
            shadow += currentDepth - bias > pcfDepth ? 0.0 : 1.0;
            //shadow += smoothstep(-0.001, 0.0, pcfDepth - (currentDepth - bias));
        }    
    }
    shadow /= 9.0; // Average the 9 samples
    
    return shadow;
}

void main() {
   // Sample G-buffer
   vec4 albedoSample = texture(gAlbedo, texCoord);
   vec4 normalSample = texture(gNormal, texCoord);
   vec4 materialSample = texture(gMaterial, texCoord);
   float depth = texture(gDepth, texCoord).r;
   
   vec3 albedo = albedoSample.rgb;
   float metallic = albedoSample.a;
   vec3 normal = normalSample.rgb * 2.0 - 1.0;  // Decode normal from [0,1] to [-1,1]
   float roughness = normalSample.a;
   vec3 fragPos = reconstructPosition(texCoord, depth);
   float emissiveStrength = materialSample.r;
   float geometryFlag = materialSample.g;
   float occlusionFactor = materialSample.b;
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
   ssaoFactor = pow(ssaoFactor, 1.0); // TEST
   
   // For directional light, use the light direction directly
   vec3 lightDir = normalize(-u_lightDir);
   vec3 viewDir = normalize(-fragPos); // Camera is at origin in view space
   
   // No distance attenuation for directional light
   float attenuation = 1.0;
   
   // Calculate shadow factor
   float bias = 0.001/(1000./50.) + length(fragPos) * 0.000004;
   //debugColor.x = length(fragPos) * 0.001;
   float shadowFactor = calculateShadow(fragPos, normal, lightDir, bias);
   
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
   
   float ff =  mix(1.0, ssaoFactor, 0.5);
   vec3 result = (ambient + (diffuse + specular) * ff * attenuation * shadowFactor) * occlusionFactor;
   result = mix(result, albedo, emissiveStrength);
   
   FragColor = vec4(result, 1.);
   //debugColor.yz = result.yz;
   //FragColor = vec4(debugColor, 1.);
}
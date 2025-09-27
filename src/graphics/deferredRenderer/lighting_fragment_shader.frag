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

float reconstructPositionZ(vec2 screenCoord, float depth) {
   // Convert screen coordinates to NDC
   vec2 ndc = screenCoord * 2.0 - 1.0;
   
   // Create clip space coordinates
   vec4 clipSpace = vec4(ndc, depth * 2.0 - 1.0, 1.0);
   
   // Extract Z and W rows from the matrix and compute only what we need
   vec4 zRow = vec4(u_inverseProjection[0].z, u_inverseProjection[1].z, u_inverseProjection[2].z, u_inverseProjection[3].z);
   vec4 wRow = vec4(u_inverseProjection[0].w, u_inverseProjection[1].w, u_inverseProjection[2].w, u_inverseProjection[3].w);
   float viewSpaceZ = dot(zRow, clipSpace);
   float viewSpaceW = dot(wRow, clipSpace);
   return viewSpaceZ / viewSpaceW;
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
      //vec3 reconstructedPos = reconstructPosition(offset.xy, texture(gDepth, offset.xy).r);
      //float sampleDepth = reconstructedPos.z;
      float sampleDepth = reconstructPositionZ(offset.xy, texture(gDepth, offset.xy).r);
      
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

vec4 calculateSSR(vec3 fragPos, vec3 normal, vec3 viewDir) {
    // Calculate reflection direction
    vec3 reflectionDir = reflect(-viewDir, normal);
    
    // Hardcode scale as 5.0
    float scale = 10.0 * (1.-dot(normal, reflectionDir));
    
    // Choose number of steps
    int numSteps = 16;
    
    // March linearly in view space along the reflection ray
    vec3 stepSize = (reflectionDir * scale) / float(numSteps);
    
    for (int i = 1; i <= numSteps; i++) {
        // Get current position in view space
        vec3 currentPos = fragPos + stepSize * float(i);
        
        // Check if ray goes behind the camera (positive Z in view space)
        if (currentPos.z > 0.0) {
            break;
        }
        
        // Project current position to screen space
        vec4 screenPos = u_projection * vec4(currentPos, 1.0);
        screenPos.xyz /= screenPos.w;
        
        // Convert to [0,1] range for sampling
        vec2 screenUV = screenPos.xy * 0.5 + 0.5;
        float projectedDepth = screenPos.z * 0.5 + 0.5;
        
        //// Check bounds
        //if (screenUV.x < 0.0 || screenUV.x > 1.0 || 
        //    screenUV.y < 0.0 || screenUV.y > 1.0) {
        //    break;
        //}

        // Calculate screen edge fade factor
        float ff = smoothstep(0.0, 0.1, screenUV.x) *
                   (1.0 - smoothstep(0.9, 1.0, screenUV.x)) *
                   smoothstep(0.0, 0.1, screenUV.y) *
                   (1.0 - smoothstep(0.9, 1.0, screenUV.y));
        
        // Calculate fresnel factor
        float fresnelFactor = 1.0 - pow(max(dot(viewDir, normal), 0.0), 1.0);
        ff *= fresnelFactor;
        
        // Skip if fade factor is too low
        if (ff < 0.01) {
            continue;
        }
        
        // Sample depth at current screen position
        float sampledDepth = texture(gDepth, screenUV).r;

        // Reconstruct world position of the sampled surface
        vec3 surfacePos = reconstructPosition(screenUV, sampledDepth);
        
        // Calculate actual view space penetration depth  
        float penetrationDepth = surfacePos.z - currentPos.z; // Both in view space
        float maxThickness = 1.6*scale/float(numSteps); // World space units - much easier to tune!
         
        // If ray has penetrated past surface but within reasonable thickness
        // (penetrationDepth > 0 means currentPos has gone past surfacePos)
        if (penetrationDepth > 0.0 && penetrationDepth < maxThickness) {
            // We've hit a surface, get the color
            vec3 reflectedColor = texture(gAlbedo, screenUV).rgb * ff;
            // Add distance fade - closer hits have more weight
            float distanceFade = float(i) / float(numSteps);
            distanceFade = 1.0 - distanceFade*distanceFade;
            float finalWeight = ff * distanceFade;
            //debugColor.r = finalWeight;
            
            return vec4(reflectedColor, finalWeight);
        }
    }
    
    // No hit found
    return vec4(0.0, 0.0, 0.0, 0.0);
}

//vec3 calculateSSR(vec3 fragPos, vec3 normal, vec3 viewDir) {
//   // Calculate view direction and reflection direction
//   vec3 reflectionDir = reflect(-viewDir, normal);
//    
//   vec3 reflectedColor = vec3(0.0);
//    
//   // March along reflection direction
//   float scale = min(1., length(fragPos));
//   vec3 samplePos = fragPos + reflectionDir;
//   
//   // Project to screen space
//   vec4 screenPosStart = u_projection * vec4(fragPos, 1.0);
//   screenPosStart.xy /= screenPosStart.w;
//   screenPosStart.xy = screenPosStart.xy * 0.5 + 0.5;
//   vec4 screenPos = u_projection * vec4(samplePos, 1.0);
//   screenPos.xy /= screenPos.w;
//   screenPos.xy = screenPos.xy * 0.5 + 0.5;
//   //screenPos.xy = 0.1*normalize(screenPos.xy - screenPosStart.xy) +
//   //   screenPosStart.xy;
//   
//   // Check bounds and sample albedo
//   //if (screenPos.x >= 0.0 && screenPos.x <= 1.0 && screenPos.y >= 0.0 && screenPos.y <= 1.0) {
//   vec3 sampledAlbedo = texture(gAlbedo, screenPos.xy).rgb;
//   reflectedColor += sampledAlbedo;
//   float ff = smoothstep(0.0, 0.1, screenPos.x) *
//              (1.-smoothstep(0.9, 1.0, screenPos.x)) *
//              smoothstep(0.0, 0.1, screenPos.y) *
//              (1.-smoothstep(0.9, 1.0, screenPos.y))
//   ;
//   ff *= min(1., length(fragPos));
//   vec3 reconstructedPos = reconstructPosition(screenPos.xy, texture(gDepth, screenPos.xy).r);
//   vec3 sampleNormal = texture(gNormal, screenPos.xy).xyz * 2.0 - 1.0;
//   vec3 hitDir = normalize(reconstructedPos - fragPos);
//   ff *= max(0., dot(-hitDir, sampleNormal));
//   ff *= max(0., dot(hitDir, reflectionDir));
//   //debugColor = vec3(test);
//   //ff = mix(ff, 1., 0.2);
//   float fresnelFactor = pow(1.-max(dot(viewDir, normal), 0.0), 1.);
//   reflectedColor *= ff * fresnelFactor;
//   //debugColor = vec3(fresnelFactor);
//   
//   return reflectedColor;
//}

void main() {
   // Sample G-buffer
   vec4 albedoSample = texture(gAlbedo, texCoord);
   vec4 normalSample = texture(gNormal, texCoord);
   vec4 materialSample = texture(gMaterial, texCoord);
   float depth = texture(gDepth, texCoord).r;
   
   vec3 albedo = albedoSample.rgb;
   float metallic = albedoSample.a;
   vec3 normal = normalize(normalSample.rgb * 2.0 - 1.0);  // Decode normal from [0,1] to [-1,1]
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
   float scale = 2048./2048. * 50./50.;
   float bias = 0.00012 * scale + length(fragPos) * 0.000002 * scale;
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
   
   // Add screen space reflections with Fresnel
   //float fresnelFactor = pow(max(dot(viewDir, normal), 0.0), 1.);
   vec4 reflectionResult = calculateSSR(fragPos, normal, viewDir);
   //debugColor.r = reflectionResult.a;
   vec3 reflectedColor = reflectionResult.rgb;
   float reflectionWeight = reflectionResult.a;
   float reflectionStrength = 0.5;
   vec3 reflectionContribution = reflectedColor * reflectionStrength * reflectionWeight;
   result += reflectionContribution * (1.0 - metallic * 0.5); // Reduce for metals to avoid over-brightening
   
   FragColor = vec4(result, 1.0);
   //debugColor.yz = result.yz;
   //FragColor = vec4(debugColor, 1.);
}
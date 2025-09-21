#version 460 core

out vec4 FragColor;

uniform sampler2D gAlbedo;
uniform sampler2D gNormal;
uniform sampler2D gPosition;
uniform sampler2D gMaterial;
uniform vec3 u_lightPos;

in vec2 texCoord;

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
   float ao = positionSample.a;
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
   
   // Calculate light and view directions (all in L-space)
   vec3 lightVec = u_lightPos - fragPos;
   vec3 lightDir = normalize(lightVec);
   vec3 viewDir = normalize(-fragPos); // Camera is at origin in L-space
   float sqrDist = dot(lightVec, lightVec);
   float attenuation = 32.0/sqrDist;
   attenuation = 1.0 - 1.0/(1.0+attenuation); // Soft max 1.
   attenuation = 1.0;
   
   // Phong lighting model
   float ambientStrength = 0.3;
   vec3 ambient = ambientStrength * albedo;
   
   float diff = max(dot(normal, lightDir), 0.0);
   diff += max(dot(normal, -lightDir), 0.0) * 0.1;
   vec3 diffuse = diff * albedo;
   
   float specularStrength = 0.5;
   vec3 reflectDir = reflect(-lightDir, normal);
   float spec = pow(max(min(dot(viewDir, reflectDir) + 0.001, 1.0), 0.0), 128.0);
   vec3 specular = specularStrength * spec * vec3(1.0);
   
   vec3 result = (ambient + (diffuse + specular) * attenuation) * ao;
   result = mix(result, albedo, emissiveStrength);
   
   FragColor = vec4(result, 1.);
}
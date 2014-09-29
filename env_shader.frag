#version 330
in vec3 normal;
uniform sampler2D matcapTexture;
void main() {
   vec3 normalized_normal = normalize(normal);
   if(normal.z<0.0f)discard;
   gl_FragColor = texture2D(matcapTexture,0.5f*normalized_normal.xy+0.5f);
 }
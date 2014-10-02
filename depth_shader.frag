#version 330
in vec3 normal;
in vec3 pos_eye;
void main() {
   	vec3 normalized_normal = normalize(normal);
    gl_FragColor = vec4(normalized_normal.xyz,pos_eye.z);
	//gl_FragColor =vec4(1,0,0,1);
	//gl_FragColor = vec4(pos_eye.z,pos_eye.z,pos_eye.z,1.0f);
 }

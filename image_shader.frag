#version 330
in vec3 pos3d;
uniform sampler2D textureImage;
uniform vec2 IMG_DIMS;
void main() {
	gl_FragColor =texture2D(textureImage,pos3d.xy);
}
 
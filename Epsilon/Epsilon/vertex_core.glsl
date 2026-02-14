#version 330 core
layout(location = 0) in vec3 aPos;
uniform vec2 screenSize;
uniform float zoom;
void main(){
 vec2 ndc;
    ndc.x = (aPos.x / screenSize.x) * 2.0 - 1.0;
    ndc.y = 1.0 - (aPos.y / screenSize.y) * 2.0;

    ndc /= zoom;
	gl_Position = vec4(ndc,aPos.z,1.0);
}
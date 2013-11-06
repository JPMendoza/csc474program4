#version 120 

#extension GL_ARB_gpu_shader5 : enable

// this is syntax for GLSL 1.2
in vec3 position;
varying vec3 cameraSpacePosition;

uniform mat4 worldSpaceMoveMatrix;
uniform mat4 worldToCameraMatrix;
uniform mat4 modelToWorldMatrix;
uniform mat4 cameraToClipMatrix;

void main()
{
   mat4 modelToCameraMatrix = worldToCameraMatrix * worldSpaceMoveMatrix * modelToWorldMatrix;
   vec4 tempCamPosition = (worldToCameraMatrix * (worldSpaceMoveMatrix * (modelToWorldMatrix * vec4(position, 1.0))));
   gl_Position = cameraToClipMatrix * tempCamPosition;
}

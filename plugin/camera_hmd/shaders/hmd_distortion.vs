#version 120
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTexCoord;
attribute vec4 aColor;
attribute vec3 aTangent;
attribute vec3 aBitangent;

varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;
void main(void)
{
    gl_TexCoord[0] = gl_TextureMatrix[0] * vec4(aTexCoord, 1.0);
    gl_Position = gl_Vertex;
};

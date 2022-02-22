varying vec4 vPosition;
varying vec3 vNormal;
varying vec3 vTexCoord;
varying vec2 vN;

uniform sampler2DShadow shadowMap;
// uniform sampler2D diffuseMap;
uniform sampler2D matcapMap;

void main(void)
{

  vec3 Ia = gl_FrontMaterial.diffuse.rgb * texture2D(matcapMap, vN).rgb;
  float gamma = 2.2;
  Ia = pow(Ia, vec3(1.0/gamma));
  vec4 shadow = shadow2DProj(shadowMap, gl_TexCoord[1]);
  gl_FragColor = vec4(Ia, shadow.a);
}

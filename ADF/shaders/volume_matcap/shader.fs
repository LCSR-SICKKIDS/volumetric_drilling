uniform vec3 uMinCorner;
uniform vec3 uMaxCorner;
uniform vec3 uTextureScale;
uniform vec3 uGradientDelta;
uniform sampler3D uVolume;
uniform sampler2DShadow shadowMap;
uniform float uIsosurface;
uniform float uResolution;

varying vec4 vPosition;
uniform bool uSmoothVolume;
uniform int uSmoothingLevel;
uniform sampler2D uMatcapMap;
uniform int uEnableShadow;

vec3 dx = vec3(uGradientDelta.x, 0.0, 0.0);
vec3 dy = vec3(0.0, uGradientDelta.y, 0.0);
vec3 dz = vec3(0.0, 0.0, uGradientDelta.z);


//----------------------------------------------------------------------
// Finds the entering intersection between a ray e1+d and the volume's
// bounding box.
//----------------------------------------------------------------------

float entry(vec3 e1, vec3 d)
{
    float t = distance(uMinCorner, uMaxCorner);

    vec3 a = (uMinCorner - e1) / d;
    vec3 b = (uMaxCorner - e1) / d;
    vec3 u = min(a, b);

    return max( max(-t, u.x), max(u.y, u.z) );
}


float isoValue(vec3 tc){
  return texture3D(uVolume, tc).a;
}

//----------------------------------------------------------------------
// Estimates the intensity gradient of the volume in model space
//----------------------------------------------------------------------

vec3 gradient(vec3 tc)
{
    vec3 nabla = vec3(
        isoValue(tc + dx) - isoValue(tc - dx),
        isoValue(tc + dy) - isoValue(tc - dy),
        isoValue(tc + dz) - isoValue(tc - dz)
    );

    return (nabla / uGradientDelta) * uTextureScale;
}


//----------------------------------------------------------------------
//  Performs interval bisection and returns the value between a and b
//  closest to isosurface. When s(b) > s(a), direction should be +1.0,
//  and -1.0 otherwise.
//----------------------------------------------------------------------

vec3 refine(vec3 a, vec3 b, float isosurface, float direction)
{
    for (int i = 0; i < 6; ++i)
    {
        vec3 m = 0.5 * (a + b);
        float v = (texture3D(uVolume, m).a - isosurface) * direction;
        if (v >= 0.0)   b = m;
        else            a = m;
    }
    return b;
}


//----------------------------------------------------------------------
//  Computes phong shading based on current light and material
//  properties.
//----------------------------------------------------------------------

vec3 shade(vec3 p, vec3 v, vec3 n)
{
    vec4 lp = gl_ModelViewMatrixInverse * gl_LightSource[0].position;
    vec3 l = normalize(lp.xyz - p * lp.w);
    vec3 h = normalize(l+v);
    float cos_i = max(dot(n, l), 0.0);
    float cos_h = max(dot(n, h), 0.0);

    vec3 Ia = gl_FrontLightProduct[0].ambient.rgb;
    vec3 Id = gl_FrontLightProduct[0].diffuse.rgb * cos_i;
    vec3 Is = gl_FrontLightProduct[0].specular.rgb * pow(cos_h, gl_FrontMaterial.shininess);

    return (Ia + Id + Is);
}


//----------------------------------------------------------------------
//  Main fragment shader code.
//----------------------------------------------------------------------

void main(void)
{
    vec4 camera = gl_ModelViewMatrixInverse * vec4(0.0, 0.0, 0.0, 1.0);
    vec3 raydir = normalize(vPosition.xyz - camera.xyz);

    float t_entry = entry(vPosition.xyz, raydir);
    t_entry = max(t_entry, -distance(camera.xyz, vPosition.xyz));

    // estimate a reasonable step size
    float t_step = distance(uMinCorner, uMaxCorner) / uResolution;
    vec3 tc_step = uTextureScale * (t_step * raydir);

    // cast the ray (in model space)
    vec4 sum = vec4(0.0);
    vec3 tc = gl_TexCoord[0].stp + t_entry * tc_step / t_step;

    vec4 dpos = vPosition;

    for (float t = t_entry; t < 0.0; t += t_step, tc += tc_step)
    {
        // sample the volume for intensity (red channel)
        float intensity = isoValue(tc);
        vec3 nabla;
        if (intensity > uIsosurface)
        {
            vec3 tcr = refine(tc - tc_step, tc, uIsosurface, 1.0);

            if (uSmoothVolume){
              // //Smoothing
              nabla = vec3(0., 0., 0.);
              vec3 tr;
              int cnt = uSmoothingLevel;
              vec3 half_vec = vec3(1., 1., 1.) * float(cnt)/2.0;
              // vec3 tcr = tc;
              for (int x = 0 ; x < cnt ; x++){
                for (int y = 0 ; y < cnt ; y++){
                  for (int z = 0 ; z < cnt ; z++){
                    tr[0] = tcr[0] + (float(x) - half_vec[0]) * uGradientDelta[0];
                    tr[1] = tcr[1] + (float(y) - half_vec[1]) * uGradientDelta[1];
                    tr[2] = tcr[2] + (float(z) - half_vec[2]) * uGradientDelta[2];
                    nabla += gradient(tr);
                  }
                }
              }
            }
            else{
              nabla = gradient(tcr);
            }

            float dt = length(tcr - tc) / length(tc_step);
            vec3 position = vPosition.xyz + (t - dt * t_step) * raydir;
            vec3 normal = normalize(nabla);
            vec3 view = raydir;

            vec3 lp = vec3(gl_LightSource[0].spotDirection);
            float bias = max(0.01 * (1.0 - dot(normal, lp)), 0.001);
            dpos.xyz = vPosition.xyz + (t - bias - dt * t_step) * raydir;

            // vec3 colour = shade(position, view, normal) * texture3D(uVolume, tcr).rgb / uIsosurface;
            vec3 e = normalize(vec3(gl_ModelViewMatrix * vec4(view, 1.0)));
            vec3 n = normalize(gl_NormalMatrix * nabla);
            vec3 r = reflect(e, n);
            float m = 2. * sqrt( pow( r.x, 2. ) + pow( r.y, 2. ) + pow( r.z + 1., 2. ) );
            vec2 vN = r.xy / m + .5;
            vec3 matcap = texture2D(uMatcapMap, vN).rgb;
            vec3 colour = matcap * texture3D(uVolume, tcr).rgb / uIsosurface;
            sum = vec4(colour, 1.0);

            // calculate fragment depth
            vec4 clip = gl_ModelViewProjectionMatrix * vec4(position, 1.0);
            gl_FragDepth = (gl_DepthRange.diff * clip.z / clip.w + gl_DepthRange.near + gl_DepthRange.far) * 0.5;

            break;
        }
    }

    // discard the fragment if no geometry was intersected
    if (sum.a <= 0.0) discard;
    if (uEnableShadow == 1){
      dpos = gl_ModelViewMatrix * dpos;
      float s = dot(gl_EyePlaneS[1], dpos);
      float t = dot(gl_EyePlaneT[1], dpos);
      float r = dot(gl_EyePlaneR[1], dpos);
      float q = dot(gl_EyePlaneQ[1], dpos);
      vec4 depos = vec4(s, t, r, q);
      vec4 shadow = shadow2DProj(shadowMap, depos);
      gl_FragColor = vec4(sum.rgb, shadow.a);
    }
    else{
      gl_FragColor = sum;
    }
}

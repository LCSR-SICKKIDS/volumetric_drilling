#version 120

//per eye texture to warp for lens distortion
uniform sampler2D warpTexture;

//Position of lens center in m (usually eye_w/2, eye_h/2)
uniform vec2 LensCenterLeft;
//Position of lens center in m (usually eye_w/2, eye_h/2)
uniform vec2 LensCenterRight;
//Scale from texture co-ords to m (usually eye_w, eye_h)
uniform vec2 ViewportScale;
//Distortion overall scale in m (usually ~eye_w/2)
uniform float WarpScale;
//Distoriton coefficients (PanoTools model) [a,b,c,d]
uniform vec4 HmdWarpParam;

//chromatic distortion post scaling
uniform vec3 aberr;

void main()
{
    //output_loc is the fragment location on screen from [0,1]x[0,1]
    vec2 output_loc = gl_TexCoord[0].xy;
    vec2 LensCenter;
    float offset;
    if (output_loc[0] <= 0.5){
      offset = 0.0;
      LensCenter = LensCenterLeft;
    }
    else{
      offset = 0.5;
      LensCenter = LensCenterRight;
    }

    output_loc[0] = (output_loc[0] - offset) * 2.0;
    //Compute fragment location in lens-centered coordinates at world scale
	  vec2 r = output_loc * ViewportScale - LensCenter;
    //scale for distortion model
    //distortion model has r=1 being the largest circle inscribed (e.g. eye_w/2)
    r /= WarpScale;

    //|r|**2
    float r_mag = length(r);
    //offset for which fragment is sourced
    vec2 r_displaced = r * (HmdWarpParam.w + HmdWarpParam.z * r_mag +
    HmdWarpParam.y * r_mag * r_mag +
    HmdWarpParam.x * r_mag * r_mag * r_mag);
    //back to world scale
    r_displaced *= WarpScale;
    //back to viewport co-ord
    vec2 tc_r = (LensCenter + aberr.r * r_displaced) / ViewportScale;
    vec2 tc_g = (LensCenter + aberr.g * r_displaced) / ViewportScale;
    vec2 tc_b = (LensCenter + aberr.b * r_displaced) / ViewportScale;

    tc_r[0] = (tc_r[0] / 2.0 ) + offset;
    tc_g[0] = (tc_g[0] / 2.0 ) + offset;
    tc_b[0] = (tc_b[0] / 2.0 ) + offset;

    float red = texture2D(warpTexture, tc_r).r;
    float green = texture2D(warpTexture, tc_g).g;
    float blue = texture2D(warpTexture, tc_b).b;
    //Black edges off the texture
    gl_FragColor = ((tc_g.x - offset < 0.0) || (tc_g.x - offset > 0.5) || (tc_g.y < 0.0) || (tc_g.y > 1.0)) ? vec4(0.0, 0.0, 0.0, 1.0) : vec4(red, green, blue, 1.0);
    // gl_FragColor = vec4(gl_TexCoord[0].xy, 0.0, 1.);
};

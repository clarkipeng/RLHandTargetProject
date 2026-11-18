// Offline frame accumulation for the showcase renderer.
// Pass 0 adds a weighted sub-frame into a float buffer (motion blur, antialiasing).
// Pass 1 resolves the buffer to display: divide by the weight sum, then dither before 8-bit quantization
// so the soft studio gradients do not band.
Shader "Hidden/Showcase/Accumulate"
{
    Properties { _MainTex ("Texture", 2D) = "white" {} }
    SubShader
    {
        ZTest Always ZWrite Off Cull Off
        CGINCLUDE
        #include "UnityCG.cginc"
        sampler2D _MainTex;
        float _Weight;
        float _Seed;
        struct v2f { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; };
        v2f vert (appdata_img v) { v2f o; o.pos = UnityObjectToClipPos(v.vertex); o.uv = v.texcoord; return o; }
        float hash(float2 p) { p = frac(p * float2(443.897, 441.423) + _Seed); p += dot(p, p.yx + 19.19); return frac((p.x + p.y) * p.x); }
        ENDCG

        Pass
        {
            Blend One One
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float4 frag (v2f i) : SV_Target { return float4(tex2D(_MainTex, i.uv).rgb * _Weight, _Weight); }
            ENDCG
        }

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            float3 LinearToSrgb(float3 c) { return c <= 0.0031308 ? 12.92 * c : 1.055 * pow(c, 1.0 / 2.4) - 0.055; }
            float4 frag (v2f i) : SV_Target
            {
                float4 acc = tex2D(_MainTex, i.uv);
                float3 c = saturate(acc.rgb / max(acc.a, 1e-6));
                // Triangular dither of one 8-bit step, applied in display space; the target is a linear (non-sRGB) 8-bit buffer.
                float3 s = LinearToSrgb(c);
                float2 px = i.pos.xy;
                float n = hash(px) + hash(px + 17.3) - 1.0;
                s += n / 255.0;
                return float4(saturate(s), 1);
            }
            ENDCG
        }
    }
}

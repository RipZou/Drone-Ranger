// Assets/Shaders/LinearEyeDepth_CMD.shader
Shader "Hidden/LinearEyeDepth_CMD"
{
    Properties
    {
        _DebugView ("Debug Grayscale (0=meters_to_R, 1=gray)", Float) = 1
        _RangeMeters ("Gray visualize range (meters)", Float) = 10
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            HLSLPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            UNITY_DECLARE_DEPTH_TEXTURE(_CameraDepthTexture);
            float _DebugView;
            float _RangeMeters;

            fixed4 frag (v2f_img i) : SV_Target
            {
        
                float raw = SAMPLE_DEPTH_TEXTURE(_CameraDepthTexture, i.uv);

                float meters = LinearEyeDepth(raw);

                if (_DebugView > 0.5)
                {
                    float vis = saturate(meters / max(_RangeMeters, 1e-4));
                    return float4(vis, vis, vis, 1);
                }

                return float4(meters, 0, 0, 1);
            }
            ENDHLSL
        }
    }
}
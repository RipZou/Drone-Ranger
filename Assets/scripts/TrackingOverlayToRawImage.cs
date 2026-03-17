// 订阅 ROS /tracking/overlay 图像，在 Canvas 的 Raw Image 上显示 tracking 结果。
// 用法：挂到任意 GameObject（如 Canvas 或 ROSConnection 下），指定 Topic 和 Raw Image；
// 若留空 Raw Image 会自动查找名为 "TrackingRawImage" 的 Raw Image。

using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class TrackingOverlayToRawImage : MonoBehaviour
{
    [Header("ROS")]
    [Tooltip("Tracking overlay image topic (tracker_node 发布 bgr8)")]
    public string topicName = "/tracking/overlay";

    [Header("Display")]
    [Tooltip("用于显示 tracking 结果的 Raw Image；留空则查找名为 TrackingRawImage 的对象")]
    public RawImage targetRawImage;

    [Header("Optional")]
    [Tooltip("收到第一帧后是否打印尺寸")]
    public bool logOnFirstFrame = true;

    ROSConnection _ros;
    Texture2D _texture;
    bool _firstFrame;
    int _lastWidth, _lastHeight;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<ImageMsg>(topicName, OnTrackingImage);

        if (targetRawImage == null)
        {
            var go = GameObject.Find("TrackingRawImage");
            if (go != null) targetRawImage = go.GetComponent<RawImage>();
            if (targetRawImage == null)
                Debug.LogWarning("[TrackingOverlayToRawImage] 未指定 Raw Image 且场景中无名为 TrackingRawImage 的对象，请在 Inspector 中指定。");
        }

        if (targetRawImage != null)
            Debug.Log($"[TrackingOverlayToRawImage] Subscribed to {topicName}, target Raw Image: {targetRawImage.name}");
    }

    void OnTrackingImage(ImageMsg msg)
    {
        if (msg.data == null || msg.data.Length == 0) return;

        int w = (int)msg.width;
        int h = (int)msg.height;
        if (w <= 0 || h <= 0) return;

        if (targetRawImage == null) return;

        bool needCreate = _texture == null || _lastWidth != w || _lastHeight != h;
        if (needCreate)
        {
            if (_texture != null) Destroy(_texture);
            _texture = new Texture2D(w, h, TextureFormat.RGB24, false, false);
            _texture.filterMode = FilterMode.Bilinear;
            _lastWidth = w;
            _lastHeight = h;
        }

        // tracker 发布的是 bgr8，转为 Unity 用的 RGB
        byte[] bgr = msg.data;
        byte[] rgb = new byte[bgr.Length];
        for (int i = 0; i < bgr.Length; i += 3)
        {
            if (i + 2 < bgr.Length)
            {
                rgb[i]     = bgr[i + 2]; // R <- B
                rgb[i + 1] = bgr[i + 1]; // G
                rgb[i + 2] = bgr[i];     // B <- R
            }
        }

        _texture.LoadRawTextureData(rgb);
        _texture.Apply();

        targetRawImage.texture = _texture;
        targetRawImage.color = Color.white;

        if (logOnFirstFrame && !_firstFrame)
        {
            _firstFrame = true;
            Debug.Log($"[TrackingOverlayToRawImage] First frame: {w}x{h}");
        }
    }

    void OnDestroy()
    {
        if (_texture != null)
        {
            Destroy(_texture);
            _texture = null;
        }
    }
}

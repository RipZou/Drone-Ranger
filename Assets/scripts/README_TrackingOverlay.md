# 在 Canvas 里增加 Tracking 结果 Raw Image

## 1. 脚本说明

- **TrackingOverlayToRawImage.cs**：订阅 ROS 话题 `/tracking/overlay`（tracker_node 发布的 bgr8 图像），把图像显示在指定的 **Raw Image** 上。

## 2. 在 Unity 里添加 Raw Image 并接上脚本

### 步骤 A：新建 Raw Image

1. 在 **Hierarchy** 里选中 **Canvas**。
2. 右键 Canvas → **UI** → **Raw Image**，得到一个新的 Raw Image 子物体。
3. 将新物体命名为 **TrackingRawImage**（脚本可自动找到；也可用别的名字，再在 Inspector 里手动指定）。

### 步骤 B：摆位置和大小

1. 选中 **TrackingRawImage**。
2. 在 **Inspector** 的 **Rect Transform** 里：
   - 建议放在与现有 depth Raw Image 不重叠的位置，例如：
     - **Anchor**: 左下角（或右下角）。
     - **Pos X/Y**: 例如 `-320, -300`（在红色 depth 下方）或 `320, -190`（在右侧）。
   - **Width / Height**: 例如 `320, 240`（与 tracking 分辨率接近即可，可后续再调）。

### 步骤 C：挂脚本并指定 Raw Image

1. 在 **Hierarchy** 里选 **Canvas**（或 **ROSConnection**，或任意常驻的 GameObject）。
2. **Add Component** → 搜索 **Tracking Overlay To Raw Image**，添加组件。
3. 在 **Inspector** 里：
   - **Topic Name**：保持 `/tracking/overlay`（与 tracker_node 发布的话题一致）。
   - **Target Raw Image**：把刚才建的 **TrackingRawImage** 拖进去；若物体名叫 `TrackingRawImage` 且未拖拽，脚本会在 Start 时自动查找。
   - **Log On First Frame**：可勾选，便于确认收到第一帧。

### 步骤 D：运行前确认

- **ROS**：先启动 `ros_tcp_endpoint`，并确保 **tracker_node** 在运行（会订阅 `/oak/rgb/image_raw` 并发布 `/tracking/overlay`）。
- **Unity**：运行场景后，ROS 连接正常且 tracker 有输出时，Tracking Raw Image 上会显示带 bbox 的 overlay 图像。

## 3. 若没有画面

- 检查 Console 是否有 `[TrackingOverlayToRawImage] First frame: WxH`，确认已收到一帧。
- 确认 **Target Raw Image** 已指定，且该 Raw Image 的 **Color** 为白色（脚本会设为 white，若被改成透明可能看不见）。
- 确认场景里 **tracker_node** 已起：  
  `ros2 run tracking tracker`  
  且 `/oak/rgb/image_raw` 有数据（Unity 的 Oak 相机在发布）。

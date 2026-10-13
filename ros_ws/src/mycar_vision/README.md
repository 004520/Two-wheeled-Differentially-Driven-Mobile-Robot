# mycar_vision

雪哥车（mycar）**纯感知**视觉前端功能包。

* **YOLO11n**（`ultralytics`）检测 `person`（COCO 0）、`bottle`（COCO 39）
* **传统 HSV 颜色分割**检测 `blue_block` / `green_block`
* **瓶贴过滤**：落在 YOLO `bottle` 框内的绿色色块会被抑制，避免瓶身绿色标签被误判成 `green_block`
* 融合后统一发布 `vision_msgs/Detection2DArray`，并可发布带框注记图
* **显示不用 Qt**：`vision_web_stream` 把带框画面转成网页 MJPEG，浏览器直接看
* 预留三个扩展节点：**单目测距/雷达融合** → **避障决策** → **Nav2 costmap 适配**

> **边界声明（重要）**
> `vision_detector` **只负责识别**：它订阅图像、发布检测结果与标注图，
> **不会**、也**不应该**发布 `/cmd_vel`。
> 下游 `avoidance_decider` 默认同样不碰 `/cmd_vel`，只输出提议 `/avoidance_cmd`。
> 任何运动控制都应由整车既有的仲裁器 / Nav2 控制器统一持有。

---

## 1. 话题一览

| 方向 | 话题 | 类型 | QoS |
| --- | --- | --- | --- |
| 订阅 | `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | BEST_EFFORT, depth 1 |
| 发布 | `/vision/detections` | `vision_msgs/Detection2DArray` | RELIABLE, depth 10 |
| 发布 | `/vision/image/compressed` | `sensor_msgs/CompressedImage` | BEST_EFFORT, depth 1 |
| 发布（可选） | `/vision/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, depth 1 |

`vision_web_stream` 再把这些话题变成 HTTP，**不产生任何新 ROS 话题**：

| 网页路径 | 数据源 |
| --- | --- |
| `/`, `/stream.mjpg`, `/snapshot.jpg` | `/vision/image/compressed` |
| `/detections.json` | `/vision/detections` |

`Detection2D.id` 携带来源信息，格式为 **`{label}#{source}#{index}`**，例如：

```
person#yolo#0
green_block#color#2
```

其中 `source ∈ {yolo, color}`。下游节点靠它区分目标来自 YOLO 还是传统视觉，
从而不需要自定义消息类型。

---

## 2. 依赖安装

```bash
# 1) ROS 2 Humble 常规依赖
sudo apt update
sudo apt install -y ros-humble-vision-msgs ros-humble-cv-bridge \
                    python3-opencv python3-numpy

# 2) YOLO11（ultralytics 自带 torch 依赖；没有 rosdep key，只能 pip 安装）
pip3 install -U ultralytics

# 3) 权重：包内已附带 models/yolo11n.pt，离线也能跑；
#    若想使用其它权重，放到包内 models/ 或改 model 参数指向绝对路径即可。

# 4) 浏览器显示（二选一，见第 6 章）
#    a) 本包自带 vision_web_stream —— 零额外依赖，跳过这步即可
#    b) 上游 web_video_server
sudo apt install -y ros-humble-web-video-server
```

> `ultralytics` 与 `cv_bridge` 各自依赖不同版本的 NumPy 时，导入 `cv_bridge` 可能打印
> `AttributeError: _ARRAY_API not found`。这是**无害**的，本包把 `cv_bridge` 改成
> **惰性导入**，只在开启 `publish_annotated_raw` 时才尝试加载。

---

## 3. 编译

```bash
cd ~/mycar_ws
colcon build --packages-select mycar_vision --symlink-install
source install/setup.bash
```

自检：

```bash
colcon list | grep mycar_vision     # 应显示 ros.ament_python
ros2 pkg executables mycar_vision   # 应列出 5 个可执行程序
```

---

## 4. 运行

### 4.1 只跑检测节点（接真实相机）

```bash
ros2 launch mycar_vision vision_detector.launch.py
# 或
ros2 run mycar_vision vision_detector --ros-args --params-file \
  $(ros2 pkg prefix mycar_vision)/share/mycar_vision/config/vision_detector.yaml
```

### 4.2 无车自测：假相机

`fake_camera_publisher` 发布与 `mycar_camera` **完全一致**的话题/类型/QoS。

```bash
# 合成场景（默认）：一蓝两绿方块的确定性画面
ros2 run mycar_vision fake_camera_publisher

# 真实图片循环
ros2 run mycar_vision fake_camera_publisher --ros-args \
  -p "images:=['/path/to/bus.jpg']" -p fps:=20.0

# 开检测
ros2 run mycar_vision vision_detector --ros-args -p log_interval:=5.0
```

### 4.3 整条流水线（感知 + 扩展点开关）

```bash
# 默认：detector + radar fusion
ros2 launch mycar_vision vision_pipeline.launch.py

# 追加 Nav2 适配器（/vision/obstacles_scan + /vision/obstacles_cloud）
ros2 launch mycar_vision vision_pipeline.launch.py enable_costmap:=true

# 追加反应式避障（只发 /avoidance_cmd，绝不发 /cmd_vel）
ros2 launch mycar_vision vision_pipeline.launch.py enable_avoidance:=true

# 追加浏览器显示（不用 Qt）
ros2 launch mycar_vision vision_pipeline.launch.py enable_web_stream:=true
```

### 4.4 浏览器看带框视频（不用 Qt）

`vision_web_stream` 把 `/vision/image/compressed` 重新包装成网页 MJPEG 流，
车上/局域网里任何浏览器都能看，不需要 Qt、不需要额外 apt 包。

```bash
# 单独起显示，然后把浏览器指向 http://<本机IP>:8081/
ros2 launch mycar_vision vision_display.launch.py

# 或者交给 vision_pipeline 一起拉起
ros2 launch mycar_vision vision_pipeline.launch.py enable_web_stream:=true

# 想看它和检测节点一起起来（含假相机自测）
ros2 launch mycar_vision vision_display.launch.py with_detector:=true
```

页面自带四路接口：

| 路径 | 内容 |
| --- | --- |
| `/` | 完整网页：实时视频 + 目标表格 + 帧率/延迟/分辨率卡片 |
| `/stream.mjpg` | `multipart/x-mixed-replace` MJPEG 流（可嵌进任何 `<img>`） |
| `/snapshot.jpg` | 最新一帧标注图（截图/存档用） |
| `/detections.json` | 当前检测结果 JSON，方便接自己的前端 |
| `/healthz` | 存活探针 |

> **为什么不用 `web_video_server`？**
> 它当然也能用（见 `vision_display.launch.py server:=web_video_server`，
> 索引页在 `http://<本机IP>:8080/`），但它是可选的 apt 依赖，且 3.1.0 对
> `sensor_msgs/CompressedImage` 的话题名解析有坑（需要 `topic=/vision/image`
> 这种去掉 `/compressed` 的写法，否则回落到 mjpeg 并报
> `Could not find compressed image topic`）。
> 本包自带的 `vision_web_stream` 只用标准库，行为完全可控，因此作为默认方案。

### 4.5 查看结果

```bash
ros2 topic hz   /vision/detections
ros2 topic hz   /vision/image/compressed
ros2 topic echo /vision/detections --once
ros2 topic echo /vision/obstacles  --once
ros2 run rqt_image_view rqt_image_view      # 备选：桌面 GUI（非必需）
```

命令行单独覆盖任意参数（不用改 YAML）：

```bash
ros2 launch mycar_vision vision_detector.launch.py \
  confidence:=0.4 image_size:=320 image_topic:=/camera/image_raw/compressed
```

> `ros2 run ... -p name:=value` 的 `value` 会按 YAML 推断类型，**浮点参数必须写成浮点**，
> 例如 `-p log_interval:=5.0`；写成 `-p log_interval:=5` 会被当成整数，
> 与声明的 double 类型冲突而抛 `InvalidParameterTypeException`
> （用本包的 `launch` 文件传参不会踩这个坑，它会按参数声明类型解析）。

---

## 5. 参数（`config/vision_detector.yaml`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `yolo11n.pt` | 权重文件名或绝对路径 |
| `model_dir` | `""` | 额外搜索目录（优先于包内 `models/`） |
| `image_size` | **416** | YOLO `imgsz`（硬性要求，勿随意改大） |
| `device` | `""` | `""`=自动，`cpu` 强制 CPU |
| `confidence` | `0.5` | YOLO 置信度阈值 |
| `iou` | `0.45` | YOLO NMS IoU 阈值 |
| `image_topic` | `/camera/image_raw/compressed` | 输入 |
| `detection_topic` | `/vision/detections` | 输出 |
| `annotated_topic` | `/vision/image/compressed` | 标注图输出 |
| `annotated_raw_topic` | `/vision/image_raw` | 原始标注图（需 cv_bridge） |
| `frame_id` | `""` | `""`=沿用输入 header 的 frame |
| `person_class_id` | `0` | COCO person |
| `bottle_class_id` | `39` | COCO bottle |
| `publish_annotated` | `true` | 是否发标注图 |
| `annotated_quality` | `70` | JPEG 质量 |
| `publish_annotated_raw` | `false` | 需 cv_bridge 可用 |
| `detect_colored_blocks` | `true` | 是否启用颜色分割 |
| `color_classes` | `["blue_block","green_block"]` | 颜色类别 |
| `color_min_area` | **800** | 色块最小面积（像素） |
| `color_max_area` | `0` | 上限，`0`=不限 |
| `color_min_fill` | `0.45` | 轮廓面积/外接框面积，滤细长噪声 |
| `color_min_solidity` | `0.55` | 轮廓面积/凸包面积 |
| `color_morph_kernel` | `5` | 形态学核大小 |
| `bottle_containment_ratio` | `0.35` | 与 bottle 框重叠超过该比例即抑制 |
| `bottle_center_margin` | `0.10` | 中心落入 bottle 框外扩比例亦抑制 |
| `log_interval` | `10.0` | 日志周期（秒） |

### 5.1 显示参数（`config/vision_web_stream.yaml`）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `image_topic` | `/vision/image/compressed` | 要推流出去的标注图 |
| `detection_topic` | `/vision/detections` | 用于页面目标表格 |
| `host` | `0.0.0.0` | 绑定地址，`0.0.0.0` 允许局域网访问 |
| `port` | `8081` | HTTP 端口（避开 `web_video_server` 的 8080） |
| `fps` | `15.0` | MJPEG 推送上限（浏览器是最慢的一环） |
| `jpeg_quality` | `75` | 重编码质量；**≥95 时直接转发原 JPEG，不再解码，CPU 最低** |
| `log_interval` | `30.0` | 日志周期（秒） |

---

## 6. 处理流程与关键实现

```
CompressedImage ──cv2.imdecode──► BGR frame
                                     │
                     ┌───────────────┴───────────────┐
                     ▼                               ▼
            YOLO11n(imgsz=416, CPU)          HSV 颜色分割
            person(0) / bottle(39)      blue_block / green_block
                     │                               │
                     └──────► bottle 框过滤 ◄────────┘
                                   │
                       Detection2DArray + 标注图
```

* **帧丢弃**：`_image_callback` 用 `busy` 标志 + `KEEP_LAST depth=1` 的 Best-Effort QoS，
  推理期间到达的新帧直接丢弃（而不是排队），保证延迟恒定、不会雪崩。
* **模型路径搜索顺序**：`model_dir` → `share/mycar_vision/models/` →
  包源码 `models/` → 当前工作目录 → `~/.cache/mycar_vision/`。
* **瓶贴过滤**：`suppress_blocks_in_containers()`，判定条件二选一——
  色块中心落入 bottle 框（外扩 `bottle_center_margin`），或色块与 bottle 框重叠
  超过 `bottle_containment_ratio`。被抑制的色块仍会以红框画在标注图上，方便调参。
* **`_to_detection_array`**：检测框按面积降序输出，`id` 内含来源；box 用
  `bbox.center.position` + `bbox.size_x/size_y` 表达。

### 6.1 显示链路（`vision_web_stream`）

```
/vision/image/compressed ─┐
                          ├─► FrameHub（带锁的"最新值"槽）──► HTTP 线程 ──► 浏览器
/vision/detections ───────┘        ▲                                 
                              ROS 回调线程只做 memcpy，绝不做 socket I/O
```

* ROS 线程与 HTTP 线程**完全分离**：回调只把最新 JPEG/检测写进 `FrameHub`，
  HTTP 线程用 `serial` 计数器判断有没有新帧，避免任何跨线程阻塞。
* MJPEG 用标准 `multipart/x-mixed-replace`，所以 `<img src="/stream.mjpg">` 直接就能播。
* 客户端关页面会产生 `BrokenPipeError`，已就地吞掉，不会刷日志也不会拖死服务。
* `jpeg_quality>=95` 时原样转发 `vision_detector` 已经编码好的 JPEG，
  **整个节点几乎零 CPU**。

---

## 7. 扩展点设计（避障融合的接口）

三个节点互相解耦，各自只依赖标准消息，可单独启停、单独替换。

### 7.1 `vision_radar_fusion` —— 距离估计与融合（扩展点 #1）

```
/vision/detections (Detection2DArray) ┐
                                      ├─► /vision/obstacles (Detection3DArray)
/scan (LaserScan)                     ┘
```

* 距离来源优先级：**雷达/激光**（盒子张角窗口内最近有效回波）→ **单目回退**
  `Z = fy · H_real / h_pixel`（按类别查 `object_heights`）。
* 输出在 `output_frame`（默认 `base_link`）下的 3D 框；`use_tf=true` 时用 tf2 把
  `camera_optical_frame → output_frame` 做变换，默认关闭（假设相机前视、与 base 对齐）。
* **坐标系约定**：相机光学系 x 右 / y 下 / z 前，机器人系（REP-103）y 朝左，
  因此横向符号取反（代码中 `a_min = -bearing_right`）。
* 需替换的参数：`fx/fy/cx/cy`（内参）、`object_heights`（真实高度）—— 已于
  2026-10-04 填入实测标定值，见 7.1.1。

#### 7.1.1 相机内参标定（2026-10-04）

**分辨率：800 × 600**。相机固件 `leap_cam/main/camera_init.c` 设
`.frame_size = FRAMESIZE_SVGA` = 800×600；配置里原先 640×480 的默认值来自
`fake_camera_publisher` 合成场景的默认尺寸，并非真实相机。

> 第一版标定误用了浏览器截屏（约 1216×924，且 15 张尺寸在 1193×898 ~ 1254×954 间浮动，
> 即 800×600 视频被放大约 1.5 倍后的窗口），得到 fx=1045.86 等属于"1216×924 空间"的值，
> **该版已作废**。现版本改用 `~/calib/capture.py` 抓取**解码后的原始帧**（尺寸恒为
> 800×600）重新标定，内参直接对应 `/vision/detections` 的像素空间，无需再缩放。

**标定结果（800×600，最终采用）**

| 项 | 值 |
| --- | --- |
| 标定板 | 9×6 格，内角点 8×5，方格 29.47 mm |
| 图像尺寸 | 800 × 600 |
| fx / fy | 977.1961 / 978.5299 |
| cx / cy | 414.7063 / 281.9281 |
| 畸变 k1 k2 p1 p2 k3 | -0.447961, 3.191076, 0.001769, -0.018750, -10.167437 |
| RMS | 0.1843 px |
| 平均重投影误差 | 0.0291 px |
| 判定 | good |

**填入 `vision_radar_fusion.yaml` 的值**

| 参数 | 原默认（640×480 估算） | 现值（800×600 实测） |
| --- | --- | --- |
| `fx` | 554.3 | 977.1961 |
| `fy` | 554.3 | 978.5299 |
| `cx` | 320.0 | 414.7063 |
| `cy` | 240.0 | 281.9281 |

自洽性校验：HFOV = 2·atan(400/fx) = **44.5°**，`(cx, cy)` = (414.71, 281.93) 接近画面
中心 (400, 300)。

畸变系数是尺度无关量，但本节点**没有畸变参数、也不做去畸变**，因此 k1…k3 未被使用。
k1 = -0.448、k3 = -10.17 量级较大，画面边缘的方位角会有明显未建模偏差；若实测发现边缘
测距/方位误差偏大，需在 `vision_detector` 里加 `cv2.undistort`
（`getOptimalNewCameraMatrix` + `initUndistortRectifyMap`）后再送检测。

同时 `object_heights` 新增 `bottle_edge:0.22` —— 此前 `mono_depth_classes` 与
`target_classes` 已含 `bottle_edge`，但缺高度会导致单目回退拿不到距离。

**单目测距验证（2026-10-04，真值 0.70 m）**

目标为可口可乐瓶（红白标签）。抓帧确认 `/camera/image_raw/compressed` 为 **800×600**
（与内参一致）；`/scan` 无数据 → 100% 走单目回退。80 个样本：

| 真值 | 测得（全部样本） | 误差 |
| --- | --- | --- |
| 0.70 m | mean **1.044 m**（median 1.182，std 0.244，min 0.659，max 1.455） | **+49.1%** |

按 YOLO 框高分档（`Z = fy · H / h_pixel`，H = 0.22 m）：

| 框高 | 占比 | 测得 | 误差 | 说明 |
| --- | --- | --- | --- | --- |
| ~325 px（整瓶） | 29% | **0.663 m** | **−5.3%** | 正确 |
| ~183 px（仅瓶盖+标签） | 69% | **1.18 m** | +69% | 只框到上半部 |

**结论**：内参（fx/fy/cx/cy）与 `bottle: 0.22` 高度是**正确**的 —— 整瓶帧误差仅 −5.3%。
误差全部来自 **YOLO 对该透明瓶的框不稳定**：逐帧实测 10 帧中 **7 帧只有局部框**、3 帧整瓶框与
局部框并存（框宽始终 ~110-120 px，只有高度在跳）。

**修复方向**（按优先级）：
1. **恢复激光雷达 `/scan`** —— `vision_radar_fusion` 的距离优先级是「雷达/激光 → 单目回退」，
   雷达一回来就不会再用这个脆弱的单目估计。这是正解。
2. 在瓶身**下半部**贴色带，给 YOLO 一个可把框延伸到瓶底的强特征（色带选蓝/青还能顺带命中
   现有 `BOTTLE` HSV 档案）。
3. 若坚持纯单目，需给 `vision_radar_fusion.py` 增加「多框取最大」或「底边接地」逻辑 ——
   当前它只有按框高一种算法（`detection_utils.MonoCameraModel.distance_from_pixel_height`）。

> 2026-10-04 更早一次尝试时相机离线：`10.232.20.234` 可 ping 通、`:80` 配网页正常
> （页面自述"STA 模式 / 视频地址 http://10.232.20.234:81/"），但 **`:81` TCP 握手超时**，
> 视频服务未起来，`http_video_publisher` 一帧都收不到。断电重启相机后 `:81` 恢复。

**恢复雷达后的结果（2026-10-04，真值 0.70 m）—— 比单目更差**

起 `bringup_all_ekf.launch.py` 后 `/scan` 恢复（9.3 Hz、360 束、`laser_frame`），融合确实切到了
雷达（日志 `radar 81 mono 0`，`id=bottle#yolo#radar#0`）。但距离**更离谱**：

| 真值 | 测得（81 样本） | 误差 |
| --- | --- | --- |
| 0.70 m | mean **3.281 m**（median 3.189，std 0.300，min 1.927，max 3.773） | **+368.8%** |

分布集中在 3.19 m 与 3.77 m 两簇 —— 那是**墙**。

**根因：相机 ↔ 雷达的角度对齐从未标定**

1. URDF（`mycar_description/urdf/mycar.urdf`）里**根本没有 camera link/joint**，只有
   `laser_joint`（`xyz="0 0 0.075"`，无 rpy）。相机相对 `base_link` 的位姿没有任何定义，
   融合只能硬假设"相机光轴 == base 前向"（`use_tf: false`）。
2. 融合把检测框的方位角窗口直接当雷达角度用（`a_min=-bearing_right, a_max=-bearing_left`），
   瓶子居中 → 窗口 ≈ **[-5°, +5.5°]**。实测该窗口内：`-5°~+2°` 全是**无效回波 0.000**，
   `+3°~+7°` 是 **3.78~3.83 m 的墙** → 20 分位取到 ≈3.2 m。360° 边界**没有 wraparound 处理**，
   所以 `359°/355°` 的回波（`359° = 0.788 m`）永远进不了窗口。
3. 雷达正前方存在 **8 束死区（-5°~+2° 无回波）**：`-6°=5.25 m`、`+3°=3.78 m` 之间整段为 0，
   疑似相机/支架在物理上遮挡了雷达正前扇区，或 `center_base_angle: 198.5` 的零点未对。

**结论：只恢复雷达不能修好测距**，必须先做「相机 ↔ 雷达偏航标定」；在此之前雷达通道反而
比单目差一个数量级。

**偏航标定与对齐修复（2026-10-04）**

1. **偏航标定**：全 360° 的 0.5–0.9 m 回波共 30 束、落在 **8 个角度簇**
   （52-55 / 82-86 / 116-118 / 157-161 / 235-238 / 272-275 / 307-311 / 359°）。
   房间杂物太多，**单次扫描无法唯一识别瓶子**。正前方扇区里唯一的回波是
   **`359° = 0.789 m`**（孤立，两侧多为 0.000）。

2. **`center_base_angle` 当前值 = `198.5`**（`params/camsense_lidar.yaml`，launch 未覆盖；
   `clockwise: true`）。驱动用法（`src/camsense_lidar.cpp:260`）：
   `angle = raw_first + raw_step*i + center_base_angle`，再归一化到 `[0,360)`；
   **无回波/无效点直接 `continue` 跳过** → 输出里的 `0.000` 表示该角度确实没有数据。

3. **修复两个真实 bug**（`vision_radar_fusion.py`）：
   - **wraparound**：`_radar_range()` 原来用 `angles >= a_min & angles <= a_max` 比较原始值，
     窗口跨 0°/360° 接缝时（正前方目标 + `angle_min=0`）会**漏掉 359° 那束**，反而取到窗口
     外侧的墙。改为比较**环绕角差** `(angles-centre+π)%2π-π`。
   - **方位角普通平均**：`theta = np.mean(valid_angles)` 对 `{359°,0°,1°}` 会算出 ≈120–180°，
     使导出的 `(x,y)` 完全错乱。改为**圆周平均** `atan2(mean(sin), mean(cos))`。

4. **修复效果（80 样本，真值 0.70 m）**：

| 阶段 | mean | median | std | 横向 y | 误差 |
| --- | --- | --- | --- | --- | --- |
| 修复前 | 3.281 m | 3.189 m | 0.300 m | — | +368.8% |
| 修 wraparound | 0.647 m | 0.339 m | 0.985 m | +1.118 m（错乱） | −7.6%（假象） |
| 再修圆周平均 | 1.488 m | **0.888 m** | 1.038 m | **+0.088 m**（正常） | +112.6% |

修复后分布：`0.8–1.0 m` 占 65%（52/80）、`3.2–3.3 m` 占 18%（14/80）。几何已正确
（`y` 回到 ±0.09 m），但距离仍不稳定。

5. **剩余误差的物理根因**：正前方存在 **8 束（约 8°）无回波带**（published 355°–362°），
   而墙就在紧邻的 `+3°~+7°`（3.78 m）。这与「相机装在雷达前方」的物理布局一致 ——
   **相机本体遮挡了雷达的正前扇区**，导致正前方目标只能靠遮挡边缘的 `359°=0.789 m`
   一束勉强命中，窗口稍一变动就跳到 3.78 m 的墙。
   → 即**对齐（`center_base_angle`）基本是对的**（正前≈published 0°），真正卡点是**遮挡**。

6. **建议的下一步**：把相机移出雷达扫描平面（抬高/后移到雷达后方/让光束从相机上/下方
   通过），消除正前 8° 阴影；或把雷达前移。硬件调整后重跑上面同一套 80 样本统计即可判定。

### 7.2 `avoidance_decider` —— 避障决策（扩展点 #2）

```
/vision/obstacles (Detection3DArray) ─► /avoidance_cmd (Twist)
                                    └─► /cmd_vel  ← 仅当 enable_cmd_vel=true
```

* 走廊策略：FOV 内目标 < `slow_distance` 减速并按横向偏移转开，< `stop_distance`
  停车并向较空一侧转向。
* **安全契约**：默认 `enable_cmd_vel=false`，本节点**绝不触碰 `/cmd_vel`**；
  只有显式设置 `enable_cmd_vel=true` 且指定 `cmd_vel_topic` 才会直发，
  且文档要求此时不得与 Nav2 控制器共用同一话题。

### 7.3 `vision_to_costmap` —— Nav2 适配（扩展点 #3）

```
/vision/obstacles (Detection3DArray) ─┬─► /vision/obstacles_scan  (LaserScan)
                                      └─► /vision/obstacles_cloud (PointCloud2)
```

* 把每个 3D 框在角度上按宽度展开成若干 beam，分箱进 `LaserScan`（默认 720 bin），
  同箱取最近；同时按命中点生成稠密 `PointCloud2`。
* 只发观测数据，**不写 costmap**——costmap 始终归 Nav2 `obstacle_layer` 所有；
  接入全局代价地图时把 `output_frame` 设为 `odom`/`map` 并保证 TF 可用。

### 7.4 分层结论

```
vision_detector ──2D检测──► vision_radar_fusion ──3D障碍──┬─► avoidance_decider ─► /avoidance_cmd ─► (整车仲裁/Nav2)
                                                          └─► vision_to_costmap ─► scan/cloud ─► Nav2 obstacle_layer
```

- 不依赖任何自定义 `.msg`（只用 `vision_msgs`/`sensor_msgs`/`geometry_msgs`），
  便于与既有避障、导航栈对接。
- 任一层都可替换：比如用真实 `depth_image` 替换单目测距、用自研控制器替换
  `avoidance_decider`，只要保持话题契约不变。
- `Detection3DArray.header.frame_id` 明确标定坐标系，便于下游统一处理。

---

## 8. 已完成的验证

| 项 | 命令 | 结果 |
| --- | --- | --- |
| 构建 | `colcon build --packages-select mycar_vision` | 成功，类型识别为 `ros.ament_python` |
| 单元测试 | `colcon test` / `pytest test/test_detection_utils.py` | 几何、瓶贴过滤、单目测距、颜色检测、端到端融合全通过 |
| 真实图（公交） | `fake_camera_publisher` + `vision_detector` | `/vision/detections` ≈ 40 Hz，检出 `person#yolo` ×3~4（score 0.84+） |
| 合成图 | 同上 | `blue_block#color` ×2、`green_block#color` ×1，≈20 fps，38~40 ms/帧，0 丢帧 |
| 标注图 | 像素统计 | HUD、caption、各色框均绘制正确 |
| **浏览器主页** | `curl http://127.0.0.1:8081/` | HTTP 200，4980 B，`text/html`，含 `<title>mycar vision</title>` 与 `stream.mjpg` |
| **检测 JSON** | `curl .../detections.json` | HTTP 200，`count=3`，`green_block#color#0`、`blue_block#color#1/#2` |
| **快照** | `curl .../snapshot.jpg` | HTTP 200，15015 B，`image/jpeg`，解码 640×480，蓝 18332 px / 绿 12809 px |
| **MJPEG 流** | `curl --max-time 4 .../stream.mjpg` | 602737 B / 4 s，40 个 `--mycarvision` 分片、40 组 SOI+EOI，即 **10 fps 稳定** |
| **真实浏览器** | Firefox `--headless --screenshot` | 1280×1100 截图成功，页面渲染出带框视频（蓝框 41236 px、绿框 31276 px），非空白 |
| **扩展链路** | detector→fusion→avoice/costmap | `/vision/obstacles` 20.0 Hz、`/avoidance_cmd` 20.0 Hz、`/vision/obstacles_scan` 20.1 Hz、`/vision/obstacles_cloud` 20.1 Hz，`3 detections → 15 scan beams / 30 cloud points` |
| **launch 开关** | `vision_pipeline.launch.py enable_costmap:=true enable_avoidance:=true` | 4 节点全 ready，日志 `cmd_vel=off`，权重从 install/share 解析成功 |

> 备注：`vision_detector` 的 `/vision/image_raw`（`publish_annotated_raw`）在本机
> 因 `cv_bridge` 的 NumPy 2 ABI 不兼容而不可用，日志会打印 `_ARRAY_API not found`。
> 默认关闭，不影响主链路（标注图走纯 `cv2.imencode`）。

---

## 9. 已知限制与下一步

**限制**

1. 单目测距仅是量级估计，`fx/fy/cx/cy` 与各类真实高度都是占位值，必须替换为实测标定。
2. 颜色分割基于固定 HSV 阈值，强光/阴影/白平衡变化会影响稳定性。
3. YOLO11n @ 416 在 CPU 上约 40 ms/帧，NPU/GPU 加速需另配 `device`。
4. 目前只在图像帧上验证，尚未上真机的相机—雷达外参与时间同步。
5. 浏览器显示默认无鉴权，仅适合车内外网；对外暴露前需加一层反向代理/口令。

**下一步计划**

1. 用棋盘格标定真实内参，更新 `config/vision_radar_fusion.yaml`。
2. 采集实车光照样本，把 HSV 阈值做成按场景可切换的 profile。
3. 用真实 `/scan` 与相机联合标定外参，打开 `use_tf=true` 并补 TF 树。
4. 与整车仲裁器约定 `/avoidance_cmd` 的优先级语义（或改接 Nav2 `velocity_smoother`）。
5. 若 CPU 仍紧张，评估导出 ONNX / 使用 NPU，并保持 `image_size=416` 的接口约束。
6. 显示页接入配网界面：ESP32-CAM 的 `http://<IP>:81/` 是原始流，本包的
   `http://<IP>:8081/` 是带框流，下一步在配网页上加一个跳转按钮，做到
   "一个界面既配网又看检测效果"。
7. 显示页加阈值滑条（`confidence` / `color_min_area`），通过 ROS 参数服务
   在线调参，免去反复重启节点。

---

## 10. 包结构

```
mycar_vision/
├── package.xml
├── setup.py / setup.cfg
├── README.md
├── resource/mycar_vision
├── config/
│   ├── vision_detector.yaml
│   ├── vision_radar_fusion.yaml
│   ├── avoidance_decider.yaml
│   ├── vision_to_costmap.yaml
│   └── vision_web_stream.yaml
├── launch/
│   ├── vision_detector.launch.py
│   ├── vision_pipeline.launch.py
│   └── vision_display.launch.py     # 浏览器显示（builtin / web_video_server / both）
├── models/
│   └── yolo11n.pt
├── test/
│   └── test_detection_utils.py
└── mycar_vision/
    ├── __init__.py
    ├── detection_utils.py        # 几何 / 颜色常量 / 单目相机模型（无 ROS 依赖）
    ├── color_detector.py         # HSV 色块检测
    ├── vision_detector.py        # ★ 主节点：YOLO11n + 颜色 + 瓶贴过滤
    ├── vision_web_stream.py      # ★ 浏览器显示：/vision/image/compressed → MJPEG 网页
    ├── vision_radar_fusion.py    # 扩展点 #1：距离估计与融合 → /vision/obstacles
    ├── avoidance_decider.py      # 扩展点 #2：避障决策 → /avoidance_cmd
    ├── vision_to_costmap.py      # 扩展点 #3：Nav2 适配 → scan / cloud
    └── fake_camera_publisher.py  # 无车自测的假相机
```

可执行程序（`ros2 run mycar_vision <name>`）：
`vision_detector`、`vision_web_stream`、`vision_radar_fusion`、`avoidance_decider`、
`vision_to_costmap`、`fake_camera_publisher`。

# AFFS — Autonomous Formation Flight System

## Simülasyonu Başlatma

### Standart (CPU)
```bash
gz sim -v4 -r affs.sdf
```

### Isaac ROS / GPU (Önerilen)
```bash
GZ_VIDEO_ALLOWED_ENCODERS=NVENC gz sim -v4 -r affs_max.sdf
```
> NVENC, Gazebo'nun H264 encode işlemini NVIDIA GPU üzerinde yapmasını sağlar.

---

## FOLLOWER ArduPlane SITL

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduPlane -f JSON --model JSON \
  --add-param-file=$HOME/SITL_Models/Gazebo/config/mini_talon_vtail.param \
  --console --map -I0 \
  --out=udp:127.0.0.1:14570 \
  --out=udp:127.0.0.1:14551 \
  --custom-location=38.700853,27.453821,10,0
```

MAVProxy açıldığında:
```
param set SYSID_THISMAV 1
param set ARMING_CHECK 0
```

---

## LEADER ArduPlane SITL

```bash
cd ~/ardupilot
sim_vehicle.py -v ArduPlane -f JSON --model JSON \
  --add-param-file=$HOME/SITL_Models/Gazebo/config/mini_talon_vtail.param \
  --console --map -I1 \
  --out=udp:127.0.0.1:14560 \
  --out=udp:127.0.0.1:14553 \
  --custom-location=38.700833,27.454009,10,0
```

---

## Kamera Stream

### 1. Streaming'i Etkinleştir

**Forward Camera (Port 5600):**
```bash
gz topic -t /world/affs/model/follower/link/base_link/sensor/forward_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p 'data: true'
```

**Right Side Camera (Port 5601):**
```bash
gz topic -t /world/affs/model/follower/link/base_link/sensor/right_side_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p 'data: true'
```

### 2. Stream'i Görüntüle

**Port 5600 — CPU (Standart):**
```bash
gst-launch-1.0 -v udpsrc port=5600 \
  caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

**Port 5601 — CPU (Standart):**
```bash
gst-launch-1.0 -v udpsrc port=5601 \
  caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

**Port 5601 — GPU (Önerilen, RTX 4050+):**
```bash
gst-launch-1.0 -q udpsrc port=5601 \
  ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 \
  ! rtph264depay ! h264parse \
  ! nvh264dec \
  ! cudaconvert ! cudadownload \
  ! video/x-raw,format=I420 \
  ! videoconvert \
  ! autovideosink sync=false
```
> Gereksinim: `gstreamer1.0-plugins-bad` + NVIDIA sürücü + `nvcodec` plugin (`gst-inspect-1.0 nvh264dec` ile kontrol et)

---

## Isaac ROS AprilTag Node

```bash
python3 /workspaces/ros2_ws/src/isaac_camera_node_fixed.py
```

Isaac ROS AprilTag detection:
```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py \
  tag_family:=tag36h11 tag_size:=0.1
```

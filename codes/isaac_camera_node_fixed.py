#!/usr/bin/env python3
"""
Isaac ROS AprilTag Detection Node — DÜZELTİLMİŞ SÜRÜM
Değişiklikler:
  1. Thread-safe mod/threshold erişimi (RLock)
  2. Frame-tag zamanlaması eşleştirmesi (timestamp farkı toleransı)
  3. calib_all_done için beklenen tag listesi (EXPECTED_TAGS)
  4. focal_y ayrı hesaplandı (VERTICAL_FOV kullanılıyor)
  5. reset_session thresholds'u da temizliyor
  6. GStreamer koptuğunda yeniden bağlanma + loglama
  7. Gereksiz time.sleep(0.005) kaldırıldı
  8. Euler açı convention'ı (Isaac ROS ZYX) yorum satırıyla belgelendi
"""

import os
os.environ["QT_X11_NO_MITSHM"] = "1"   # X11 MIT-SHM BadAccess hatasını önler

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
import numpy as np
import subprocess
import cv2
import time
import threading
import json
from cv_bridge import CvBridge

# ─────────────────────────────────────────────
#  AYARLAR
# ─────────────────────────────────────────────
IMAGE_WIDTH    = 1280
IMAGE_HEIGHT   = 720
HORIZONTAL_FOV = 1.134
# DÜZELTME 4: Dikey FOV ayrı tanımlandı.
# Kare piksel varsayımıyla hesaplandı; sensörünüz farklıysa değiştirin.
VERTICAL_FOV   = HORIZONTAL_FOV * (IMAGE_HEIGHT / IMAGE_WIDTH)

CALIB_FILE     = "apriltag_thresholds.json"
SKIP_COUNT     = 10
SAMPLE_COUNT   = 10

# DÜZELTME 3: Hangi tag ID'lerini beklediğinizi buraya yazın.
# Boş bırakılırsa "görülen tag'lerin tamamı" mantığı korunur (eski davranış).
EXPECTED_TAGS: set[str] = set()  # örn: {"0", "1", "2"}

# Aynı frame'e ait sayılabilmesi için tag-frame zaman toleransı (saniye)
TAG_FRAME_TIME_TOLERANCE = 0.1

focal_x  = (IMAGE_WIDTH  / 2.0) / np.tan(HORIZONTAL_FOV / 2.0)
focal_y  = (IMAGE_HEIGHT / 2.0) / np.tan(VERTICAL_FOV   / 2.0)  # DÜZELTME 4
center_x = IMAGE_WIDTH  / 2.0
center_y = IMAGE_HEIGHT / 2.0

GST_CMD = [
    'gst-launch-1.0', '-q', 'udpsrc', 'port=5601',
    'buffer-size=2097152',
    '!', 'application/x-rtp,media=video,clock-rate=90000,encoding-name=H264',
    '!', 'rtph264depay', '!', 'h264parse',
    '!', 'nvh264dec',
    '!', 'cudaconvert', '!', 'cudadownload',
    '!', f'video/x-raw,format=I420,width={IMAGE_WIDTH},height={IMAGE_HEIGHT}',
    '!', 'videoconvert',
    '!', f'video/x-raw,format=BGR,width={IMAGE_WIDTH},height={IMAGE_HEIGHT}',
    '!', 'fdsink'
]

# ─────────────────────────────────────────────
#  TAG ID YARDIMCISI
# ─────────────────────────────────────────────
def parse_tid(raw_id):
    if hasattr(raw_id, '__iter__'):
        return str(int(raw_id[0]))
    return str(int(raw_id))


# ─────────────────────────────────────────────
#  GSTREAMER PROCESS YÖNETİCİSİ  (DÜZELTME 6)
# ─────────────────────────────────────────────
def _start_gst_process(frame_size: int) -> subprocess.Popen:
    return subprocess.Popen(
        GST_CMD, stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, bufsize=frame_size)


class IsaacCameraNode(Node):
    def __init__(self):
        super().__init__('isaac_camera_node')
        self.image_pub = self.create_publisher(Image,      '/image',       10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.bridge    = CvBridge()
        self.frame_size = IMAGE_WIDTH * IMAGE_HEIGHT * 3

        self.latest_frame       = None
        self.latest_frame_time  = 0.0   # DÜZELTME 2: frame zaman damgası
        self.latest_tags        = []
        self.latest_tags_time   = 0.0   # DÜZELTME 2: tag zaman damgası

        # DÜZELTME 1: Tüm paylaşılan veriler için tek RLock
        self._lock = threading.RLock()

        self.mode           = 1
        self.thresholds     = {}
        self.calib_data     = {}
        self.calib_all_done = False

        self.fps         = 0.0
        self.fps_counter = 0
        self.fps_timer   = time.time()
        self.frame_count = 0

        self.create_subscription(
            AprilTagDetectionArray, '/tag_detections', self._tag_cb, 10)

        self.ci = CameraInfo()
        self.ci.width, self.ci.height = IMAGE_WIDTH, IMAGE_HEIGHT
        self.ci.distortion_model = 'plumb_bob'
        self.ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.ci.k = [focal_x, 0.0, center_x,
                     0.0, focal_y, center_y,
                     0.0, 0.0, 1.0]
        self.ci.r = [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
        self.ci.p = [focal_x, 0.0, center_x, 0.0,
                     0.0, focal_y, center_y, 0.0,
                     0.0, 0.0, 1.0, 0.0]

        self.running = True
        self.process = _start_gst_process(self.frame_size)
        threading.Thread(target=self._ros_loop, daemon=True).start()
        self.get_logger().info("IsaacCameraNode hazır.")

    # ── Tag callback ──────────────────────────────────────────────────────────
    def _tag_cb(self, msg):
        with self._lock:                          # DÜZELTME 1
            self.latest_tags      = list(msg.detections)
            self.latest_tags_time = time.time()  # DÜZELTME 2

    # ── GStreamer okuma döngüsü ───────────────────────────────────────────────
    def _ros_loop(self):
        consecutive_errors = 0
        while self.running:
            raw = self.process.stdout.read(self.frame_size)

            # DÜZELTME 6: Eksik/hatalı frame → loglama + yeniden bağlanma
            if len(raw) != self.frame_size:
                consecutive_errors += 1
                self.get_logger().warn(
                    f"GStreamer: Beklenenden kısa frame "
                    f"({len(raw)}/{self.frame_size} byte), "
                    f"hata #{consecutive_errors}")
                if consecutive_errors >= 5:
                    self.get_logger().error(
                        "GStreamer art arda 5 hata — yeniden bağlanılıyor...")
                    try:
                        self.process.terminate()
                        self.process.wait(timeout=3)
                    except Exception:
                        pass
                    self.process = _start_gst_process(self.frame_size)
                    consecutive_errors = 0
                    time.sleep(1.0)
                continue

            consecutive_errors = 0
            frame = (np.frombuffer(raw, dtype=np.uint8)
                       .reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3)).copy())
            now = self.get_clock().now().to_msg()

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = now
            img_msg.header.frame_id = 'camera'
            self.ci.header.stamp    = now
            self.ci.header.frame_id = 'camera'
            self.image_pub.publish(img_msg)
            self.info_pub.publish(self.ci)

            with self._lock:                      # DÜZELTME 1
                self.latest_frame      = frame
                self.latest_frame_time = time.time()  # DÜZELTME 2
                self.fps_counter      += 1            # Gerçek kamera frame'i geldi

    # ── Euler dönüşümü  ───────────────────────────────────────────────────────
    # NOT (DÜZELTME 8): Isaac ROS quaternion'ları kamera frame'indedir (Z ileri).
    # Aşağıdaki ZYX sırası (yaw-pitch-roll) bu convention için uygundur.
    def _quat_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2)))
        pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0)))
        yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2)))
        return roll, pitch, yaw

    # ── Oturum sıfırlama  ─────────────────────────────────────────────────────
    def reset_session(self):
        with self._lock:                          # DÜZELTME 1
            self.calib_data     = {}
            self.calib_all_done = False
            self.frame_count    = 0
            self.fps            = 0.0
            self.fps_counter    = 0
            self.fps_timer      = time.time()
            # DÜZELTME 5: Mod 1'e geçişte eski threshold'lar temizlendi
            self.thresholds     = {}

    # ── Kalibrasyon tamamlanma kontrolü  ──────────────────────────────────────
    def _check_calib_complete(self):
        """
        DÜZELTME 3: EXPECTED_TAGS doluysa yalnızca o tag'ler beklenir.
        Boşsa eskiden olduğu gibi 'görülen tag'lerin tamamı' mantığı çalışır.
        """
        if not self.calib_data:
            return False
        if EXPECTED_TAGS:
            return all(
                self.calib_data.get(t, {}).get("done", False)
                for t in EXPECTED_TAGS
            )
        return all(v["done"] for v in self.calib_data.values())

    # ── Ana görüntü işleme + çizim  ───────────────────────────────────────────
    def process_and_draw(self):
        with self._lock:                          # DÜZELTME 1
            if self.latest_frame is None:
                return None
            img            = self.latest_frame.copy()
            frame_t        = self.latest_frame_time
            tags           = list(self.latest_tags)
            tags_t         = self.latest_tags_time
            mode           = self.mode
            thresholds     = dict(self.thresholds)
            calib_all_done = self.calib_all_done

        # DÜZELTME 2: Tag ve frame zamanı çok farklıysa uyar
        time_diff = abs(frame_t - tags_t)
        if tags and time_diff > TAG_FRAME_TIME_TOLERANCE:
            tags = []   # eski tag verisini bu frame'de kullanma

        # ── FPS ───────────────────────────────────────────────────────────────
        self.frame_count += 1
        elapsed = time.time() - self.fps_timer
        if elapsed >= 1.0:
            self.fps         = self.fps_counter / elapsed
            self.fps_counter = 0
            self.fps_timer   = time.time()

        # ── HUD ───────────────────────────────────────────────────────────────
        cv2.putText(img, f"Frame: {self.frame_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(img, f"FPS: {self.fps:.1f}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        cv2.putText(img, f"Tags: {len(tags)}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0) if tags else (255, 255, 255), 2)
        mode_label = "MOD: KALIBRASYON" if mode == 1 else "MOD: UCUS"
        mode_color = (0, 165, 255)      if mode == 1 else (0, 255, 100)
        cv2.putText(img, mode_label,
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
        if mode == 1 and calib_all_done:
            cv2.putText(img, "KALIBRASYON TAMAMLANDI",
                        (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # ── Her tag işle ──────────────────────────────────────────────────────
        for det in tags:
            tid = parse_tid(det.id)
            pos = det.pose.pose.pose.position
            ori = det.pose.pose.pose.orientation
            x, y, z = pos.x, pos.y, pos.z

            if z <= 0:
                continue

            roll, pitch, yaw_deg = self._quat_to_euler(ori)

            u = int(focal_x * x / z + center_x)
            v = int(focal_y * y / z + center_y)
            u = max(10, min(IMAGE_WIDTH  - 10, u))
            v = max(25, min(IMAGE_HEIGHT - 10, v))

            cv2.circle(img, (u, v), 8, (0, 0, 255), -1)
            cv2.putText(img, f"ID:{tid}",
                        (u - 30, v - 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)

            # ── MOD 1: KALİBRASYON ───────────────────────────────────────────
            if mode == 1:
                with self._lock:                  # DÜZELTME 1: calib_data yazımı
                    if tid not in self.calib_data:
                        self.calib_data[tid] = {"skip": 0, "samples": [], "done": False}
                    cd = self.calib_data[tid]

                    if not cd["done"]:
                        if cd["skip"] < SKIP_COUNT:
                            cd["skip"] += 1
                            self.get_logger().info(
                                f"ID:{tid} SKIP {cd['skip']}/{SKIP_COUNT}")
                        else:
                            cd["samples"].append([x, y, z])
                            n = len(cd["samples"])
                            self.get_logger().info(
                                f"ID:{tid} ÖRNEK {n}/{SAMPLE_COUNT}  "
                                f"x={x:.4f} y={y:.4f} z={z:.4f}")

                            if n >= SAMPLE_COUNT:
                                arr = np.array(cd["samples"])
                                mx  = float(np.mean(arr[:, 0]))
                                my  = float(np.mean(arr[:, 1]))
                                mz  = float(np.mean(arr[:, 2]))
                                self.thresholds[tid] = {"x": mx, "y": my, "z": mz}
                                cd["done"] = True
                                self.get_logger().info(
                                    f"✓ Tag {tid} → X={mx:.4f} Y={my:.4f} Z={mz:.4f}")

                                # DÜZELTME 3: Gelişmiş tamamlanma kontrolü
                                if self._check_calib_complete():
                                    try:
                                        with open(CALIB_FILE, "w") as f:
                                            json.dump(self.thresholds, f, indent=2)
                                        self.get_logger().info(
                                            f"✓ '{CALIB_FILE}' kaydedildi:\n"
                                            + json.dumps(self.thresholds, indent=2))
                                    except Exception as e:
                                        self.get_logger().error(
                                            f"Dosya yazma hatası: {e}")
                                    self.calib_all_done = True

                    done_flag = cd["done"]

                if done_flag:
                    cv2.putText(img, "KALIB OK",
                                (u - 40, v + 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                cv2.putText(img, f"X:{x:.3f} Y:{y:.3f} Z:{z:.3f}",
                            (u - 80, v + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                cv2.putText(img,
                            f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw_deg:.1f}",
                            (u - 80, v + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1)

            # ── MOD 2: UÇUŞ  (6DOF) ──────────────────────────────────────────
            else:
                if tid in thresholds:
                    t  = thresholds[tid]
                    dx = x - t["x"]
                    dy = y - t["y"]
                    dz = z - t["z"]
                else:
                    dx, dy, dz = x, y, z
                    self.get_logger().warn(
                        f"Tag {tid} için kalibrasyon yok — ham değer.")

                self.get_logger().info(
                    f"\n--- Tag ID:{tid} ---\n"
                    f"  dx={dx:.4f}  dy={dy:.4f}  dz={dz:.4f}\n"
                    f"  Roll={roll:.1f}°  Pitch={pitch:.1f}°  Yaw={yaw_deg:.1f}°")

                cv2.putText(img, f"dX:{dx:.3f} dY:{dy:.3f} dZ:{dz:.3f}",
                            (u - 80, v + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                cv2.putText(img,
                            f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw_deg:.1f}",
                            (u - 80, v + 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0), 1)

        return img

    def destroy_node(self):
        self.running = False
        try:
            self.process.terminate()
            self.process.wait(timeout=5)
        except Exception:
            pass
        super().destroy_node()


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
def main():
    rclpy.init()
    node = IsaacCameraNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    try:
        while rclpy.ok():
            print("\n1 → Kalibrasyon | 2 → Uçuş | q → Çıkış")
            choice = input("Seçim: ").strip().lower()

            if choice == 'q':
                break
            if choice not in ('1', '2'):
                continue

            with node._lock:                      # DÜZELTME 1
                node.mode = int(choice)
            node.reset_session()

            if node.mode == 2:
                if not os.path.exists(CALIB_FILE):
                    print("HATA: Önce kalibrasyon yapın!")
                    continue
                with open(CALIB_FILE, "r") as f:
                    loaded = json.load(f)
                with node._lock:                  # DÜZELTME 1
                    node.thresholds = loaded
                print(f"Kalibrasyon yüklendi: {json.dumps(loaded, indent=2)}")

            print(f"\n{'─'*50}")
            print(f"  {'KALIBRASYON' if node.mode==1 else 'UÇUŞ'} MODU — çıkmak için 'q'")
            print(f"{'─'*50}")

            while rclpy.ok():
                display = node.process_and_draw()
                if display is not None:
                    cv2.imshow('Isaac ROS GPU - AprilTag Detection', display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    raise KeyboardInterrupt

                # DÜZELTME 7: Gereksiz time.sleep(0.005) kaldırıldı
                # cv2.waitKey(1) zaten ~1ms bekliyor.

                with node._lock:
                    all_done = node.calib_all_done
                if node.mode == 1 and all_done:
                    print("Kalibrasyon tamamlandı, 2 sn sonra mod seçimine dönülüyor...")
                    time.sleep(2)
                    break

    except KeyboardInterrupt:
        print("\nKullanıcı tarafından sonlandırıldı.")
    except Exception as e:
        print(f"\nHata: {e}")
        import traceback; traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        print("Program sonlandırıldı.")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import cv2
import numpy as np
import subprocess
from pupil_apriltags import Detector

# Gazebo kamera parametreleri (model.sdf'den)
image_width = 1280
image_height = 720
horizontal_fov = 1.134

# Kamera intrinsic parametrelerini hesapla
focal_x = (image_width / 2.0) / np.tan(horizontal_fov / 2.0)
focal_y = focal_x
center_x = image_width / 2.0
center_y = image_height / 2.0

camera_params = [focal_x, focal_y, center_x, center_y]
print(f"Kamera Parametreleri: fx={focal_x:.2f}, fy={focal_y:.2f}, cx={center_x:.2f}, cy={center_y:.2f}")

# AprilTag boyutu (metre cinsinden)
tag_size = 0.165  # 16.5 cm

# AprilTag detector oluştur
at_detector = Detector(
    families='tag36h11',
    nthreads=2,
    quad_decimate=2.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

# GStreamer pipeline ile stream'i oku
gst_cmd = [
    'gst-launch-1.0',
    '-q',
    'udpsrc', 'port=5601',
    '!', 'application/x-rtp,media=video,clock-rate=90000,encoding-name=H264',
    '!', 'rtph264depay',
    '!', 'h264parse',
    '!', 'avdec_h264',
    '!', 'videoconvert',
    '!', f'video/x-raw,format=BGR,width={image_width},height={image_height}',
    '!', 'fdsink'
]

print("GStreamer subprocess başlatılıyor...")
print(f"Komut: {' '.join(gst_cmd)}\n")

try:
    process = subprocess.Popen(
        gst_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=image_width * image_height * 3
    )
except Exception as e:
    print(f"HATA: GStreamer başlatılamadı: {e}")
    print("gst-launch-1.0 kurulu mu kontrol edin: gst-launch-1.0 --version")
    exit(1)

print("Stream başarıyla açıldı. AprilTag tespiti başlıyor...")
print("Çıkmak için 'q' tuşuna basın.\n")

frame_count = 0
frame_size = image_width * image_height * 3

try:
    while True:
        # GStreamer'dan raw frame oku
        raw_frame = process.stdout.read(frame_size)
        
        if len(raw_frame) != frame_size:
            print("Stream sonlandı veya bağlantı kesildi.")
            break
        
        # Raw bytes'ı numpy array'e çevir (yazılabilir kopya oluştur)
        img = np.frombuffer(raw_frame, dtype=np.uint8).reshape((image_height, image_width, 3)).copy()
        
        frame_count += 1
        
        # Gri tonlamaya çevir
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # AprilTag tespiti
        tags = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=tag_size
        )
        
        # Tespit edilen tag bilgilerini göster
        info_y = 30
        cv2.putText(img, f"Frame: {frame_count}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25
        cv2.putText(img, f"Tags Detected: {len(tags)}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if len(tags) > 0 else (255, 255, 255), 2)
        
        # Her tespit edilen tag için
        for tag in tags:
            print(f"\n--- Tag ID: {tag.tag_id} ---")
            print(f"Merkez: ({tag.center[0]:.2f}, {tag.center[1]:.2f})")
            print(f"Pose Translation (x,y,z): {tag.pose_t.flatten()}")
            
            # Tag köşelerini çiz (yeşil)
            corners = tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(img, pt1, pt2, (0, 255, 0), 3)
            
            # Tag merkezini çiz (kırmızı)
            center = tuple(tag.center.astype(int))
            cv2.circle(img, center, 5, (0, 0, 255), -1)
            
            # Tag ID'yi yaz
            cv2.putText(img, f"ID: {tag.tag_id}", 
                       (center[0] - 30, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # 6DoF bilgilerini ekranda göster
            pose_text_y = center[1] + 30
            
            # Translation (pozisyon)
            x, y, z = tag.pose_t.flatten()
            cv2.putText(img, f"X:{x:.2f}m Y:{y:.2f}m Z:{z:.2f}m", 
                       (center[0] - 80, pose_text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Rotation matrix'ten Euler açıları hesapla
            rotation_matrix = tag.pose_R
            
            roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = np.arctan2(-rotation_matrix[2, 0], 
                              np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)
            
            print(f"Euler Angles - Roll:{roll_deg:.1f}° Pitch:{pitch_deg:.1f}° Yaw:{yaw_deg:.1f}°")
            
            cv2.putText(img, f"R:{roll_deg:.1f} P:{pitch_deg:.1f} Y:{yaw_deg:.1f}", 
                       (center[0] - 80, pose_text_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Görüntüyü göster
        cv2.imshow('Gazebo Right Side Camera - AprilTag Detection', img)
        
        # q tuşuna basılırsa çık
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nKullanıcı tarafından sonlandırıldı.")
except Exception as e:
    print(f"\nHata: {e}")
finally:
    process.terminate()
    process.wait()
    cv2.destroyAllWindows()
    print("Program sonlandırıldı.")

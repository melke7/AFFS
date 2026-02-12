# AFFS

gz sim -v4 -r affs.sdf


-----------FOLLOWER-----------------

bashcd ~/ardupilot

sim_vehicle.py -v ArduPlane -f JSON --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/mini_talon_vtail.param --console --map -I0 --out=udp:127.0.0.1:14570 --out=udp:127.0.0.1:14551 --custom-location=38.700853,27.453821,10,0
```

**MAVProxy açıldığında:**
```
param set SYSID_THISMAV 1
param set ARMING_CHECK 0

-----------LEADER-----------------
bashcd ~/ardupilot

sim_vehicle.py -v ArduPlane -f JSON --model JSON --add-param-file=$HOME/SITL_Models/Gazebo/config/mini_talon_vtail.param --console --map -I1 --out=udp:127.0.0.1:14560 --out=udp:127.0.0.1:14553 --custom-location=38.700833,27.454009,10,0



-----------STREAM-----------------

1.Enable Streaming

--Forward Camera--

gz topic -t /world/affs/model/follower/link/base_link/sensor/forward_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p 'data: true'
  
  
--Right Side Camera--

gz topic -t /world/affs/model/follower/link/base_link/sensor/right_side_camera/image/enable_streaming \
  -m gz.msgs.Boolean -p 'data: true'
  
  
------- Display the streamed video--------

For Port 5600:

gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
  
  
For Port 5601:
  
gst-launch-1.0 -v udpsrc port=5601 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96" \
  ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  


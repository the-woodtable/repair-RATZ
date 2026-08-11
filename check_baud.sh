#!/bin/bash
# Run before flashing. Confirms the camera firmware and the S3 agree.
cd "$(dirname "$0")"
a=$(grep -oP 'Serial\.begin\(\K[0-9]+' camera_firmware/camera_firmware.ino | head -1)
b=$(grep -oP 'CAM_BAUD = \K[0-9]+' "THIS ONE/include/StereoCameras.h")
echo "camera_firmware.ino : $a"
echo "StereoCameras.h     : $b"
if [ "$a" = "$b" ]; then
  echo "MATCH — flash all THREE devices: left cam, right cam, S3"
else
  echo "*** MISMATCH *** telemetry will work, frames will stay at 0"
  exit 1
fi

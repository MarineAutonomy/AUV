echo "🔄 Deactivating sensors..."

nohup docker exec auv pkill -u mavlab -9 -f "sbg|ping|dvl|frontcam|bottomcam|modem|bar30|arduino_ps|sidescan|frontscan|ros2" >/dev/null 2>&1 &
disown

sleep 1

docker exec auv bash -ic 'ros2 daemon stop' > /dev/null 2>&1

echo "🛑 All sensors deactivated!"
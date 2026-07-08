echo "🔄 Activating sensors..."

docker exec -d auv bash -ic 'source ~/.bashrc; dvl & ping360 & ping2 & sbg & frontcam & bottomcam & modem & bar30 & sidescan'

sleep 5

echo "🟢 All sensors activated!"


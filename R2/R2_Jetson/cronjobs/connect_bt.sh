#!/bin/bash 
s=$(grep -i ps4 /home/robocon/cronjobs/details.txt)
len=${#s}
DEVICE=${s:4:len-4}
echo $DEVICE


connect_wifi() {
	echo "-----<<<<-----"
	echo $s
	echo $DEVICE
	if bluetoothctl info "$DEVICE" | grep -iq "Connected: yes"; then 
		echo "$DEVICE Device is already connected."
	else
bluetoothctl<< EOF
scan on
connect $DEVICE
EOF
	fi
	echo "----->>>>-----"
}

# Initial connection attempt
connect_wifi

#Keep checking every 10 seconds
while true; do
    sleep 10
    connect_wifi
done


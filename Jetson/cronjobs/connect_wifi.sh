#!/bin/bash

# Read WiFi SSID from details.txt
s=$(grep -i wifi /home/robocon/cronjobs/details.txt)
len=${#s}
wifi=${s:5:len-5}  

# Function to check and connect to WiFi
connect_wifi() {
    # Check if already connected
    echo "-----<<<<-----"
    echo $s
    echo $wifi
    if nmcli -t -f WIFI g | grep -q "enabled"; then
        if ! nmcli -t -f ACTIVE,SSID dev wifi | grep -q "^yes:$wifi$"; then
            echo "Not connected to $wifi. Attempting to reconnect..."
            sudo nmcli con up "$wifi"
        else
            echo "Already connected to $wifi."
        fi
    else
        echo "WiFi is disabled. Enabling WiFi..."
        sudo nmcli radio wifi on
        sleep 2
        sudo nmcli con up "$wifi"
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


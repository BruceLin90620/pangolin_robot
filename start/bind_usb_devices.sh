#!/bin/bash

# Define the udev rule file path
UDEV_RULE_FILE="/etc/udev/rules.d/99-usb-serial.rules"

# Create or overwrite the udev rule file with the following content
sudo bash -c "cat > $UDEV_RULE_FILE" <<EOL
# Udev rules for USB Serial Devices

# FT232H Device
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014",ATTRS{serial}=="FT8J0QYS" SYMLINK+="ttyDynamixel"

# CP210x UART Bridge
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001" SYMLINK+="ttyIMU"
EOL

# Reload udev rules to apply the new configurations
sudo udevadm control --reload-rules
sudo udevadm trigger

# Inform the user
echo "Udev rules have been successfully updated."
echo "You can now access the devices using /dev/ttyDynamixel and /dev/ttyIMU"

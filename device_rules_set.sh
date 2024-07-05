#!/bin/bash
set -euxo pipefail

HOME_DIR="${HOME}"
device_rules_path=/etc/udev/rules.d/

cd "$(dirname "$0")"

if [ -e /etc/udev/rules.d/13_FTDI_Serial.rules ]; then
	sudo rm /etc/udev/rules.d/13_FTDI_Serial.rules
fi
sudo cp ./USB-to-Quad-RS-485-Conv-Module/13_FTDI_Serial.rules $device_rules_path

if [ -e /etc/udev/rules.d/18_FT4232H.rules ]; then
	sudo rm /etc/udev/rules.d/18_FT4232H.rules
fi
sudo cp ./USB-to-Quad-RS-485-Conv-Module/18_FT4232H.rules $device_rules_path

sudo udevadm control --reload-rules && udevadm trigger

cat << EOS
=========================================
device rules setup success!!!
=========================================
EOS

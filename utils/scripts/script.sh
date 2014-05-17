sudo echo "ACTION==\"add\",KERNEL==\"ttyUSB[0-9]*\", GROUP=\"plugdev\", MODE=\"0666\"" >/etc/udev/rules.d/80-tty.rules
sudo echo "ACTION==\"add\",KERNEL==\"ttyACM[0-9]*\", GROUP=\"plugdev\", MODE=\"0666\"" >/etc/udev/rules.d/80-ttyacm.rules

sudo udevadm control --reload-rules

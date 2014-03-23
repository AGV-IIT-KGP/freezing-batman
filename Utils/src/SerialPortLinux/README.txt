sudo gedit /etc/udev/rules.d/80-ttyusb.rules

KERNEL=="ttyUSB[0-9]", GROUP="plugdev", MODE="0666"

service udev restart


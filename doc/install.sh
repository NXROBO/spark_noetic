echo 'Spark driver is installing'

echo 'Setting udev rules'
BASEPATH=$(cd `dirname $0`; pwd)
sudo cp $BASEPATH/rules/3ilidar-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/uarm-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/spark-usb-serial.rules /etc/udev/rules.d/
sudo cp $BASEPATH/rules/orbbec-usb.rules /etc/udev/rules.d/556-orbbec-usb.rules
sudo $BASEPATH/rules/eai_gx.sh
sudo udevadm trigger

echo 'Spark driver is installed'


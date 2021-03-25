#!/bin/bash

echo "The module jc42 is the Linux driver for MCP9808"
echo "It is loaded from /etc/modules"
echo
echo "Root user needs to send: echo jc42 0x18 > /sys/class/i2c-adapter/i2c-1/new_device"
echo "(first do 'sudo su')"
echo "(this is done as a systemd service MCP9808.service, see 'systemctl status MCP9808.service')"
echo ""
echo "Then the file /sys/class/hwmon/hwmon6/temp1_input is created and can be read by any program."
echo "This all is done already."
echo
echo "Value of /sys/class/hwmon/hwmon6/temp1_input:"
cat /sys/class/hwmon/hwmon6/temp1_input
echo "(milliCelsius - divide by 1000 to get Celsius)"
echo
echo "To get control back from kernel, that means, to use your own driver, do:"
echo "sudo su"
echo "echo 0x18 > /sys/class/i2c-adapter/i2c-1/delete_device"

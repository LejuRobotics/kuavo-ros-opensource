#!/bin/bash

sudo rm /etc/udev/rules.d/hipnuc_imu_serial.rules
sudo service udev reload
sudo service udev restart

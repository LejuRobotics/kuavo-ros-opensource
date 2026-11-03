#!/bin/bash

sudo cp hipnuc_imu_serial.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart

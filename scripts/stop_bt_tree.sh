#!/bin/bash
rosservice call /grab_box_demo/control_bt "data: false"
rosservice call /grab_box_demo/reset_bt "data: true"

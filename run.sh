#!/usr/bin/env bash

sh build.sh
. install/setup.sh
ros2 run gui gui

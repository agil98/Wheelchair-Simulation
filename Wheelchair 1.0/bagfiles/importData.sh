#!/bin/bash
for topic in `rostopic list -b imu.bag` ; do rostopic echo -p -b imu.bag $topic > Quat${topic//\//_}.csv ; done
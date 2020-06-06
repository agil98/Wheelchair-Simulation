#!/bin/bash
for topic in `rostopic list -b imu.bag` ; do rostopic echo -p -b imu.bag $topic >${topic//\//_}.csv ; done
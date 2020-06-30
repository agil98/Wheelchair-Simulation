#!/bin/bash
for topic in `rostopic list -b data.bag` ; do rostopic echo -p -b data.bag $topic > ${topic//\//_}.xlsx ; done
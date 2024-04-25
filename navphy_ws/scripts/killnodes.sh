#!/bin/bash
string=`ros2 node list`
rosstring=`echo "$string" | sed 's/\///g'`

for i in $rosstring; do
echo $i
echo `pgrep $i`
# ros2 lifecycle set /$i shutdown
# ros2 lifecycle set $i shutdown
done

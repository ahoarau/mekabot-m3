#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/meka/mekabot/m3meka/ros/shm_led_mouth/devel', type 'exit' to leave"
  . "/home/meka/mekabot/m3meka/ros/shm_led_mouth/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/meka/mekabot/m3meka/ros/shm_led_mouth/devel'"
else
  . "/home/meka/mekabot/m3meka/ros/shm_led_mouth/devel/setup.sh"
  exec "$@"
fi

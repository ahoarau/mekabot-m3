#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/meka/mekabot/m3meka/ros/shm_led_mouth/catkin_generated', type 'exit' to leave"
  . "/home/meka/mekabot/m3meka/ros/shm_led_mouth/catkin_generated/setup_cached.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/meka/mekabot/m3meka/ros/shm_led_mouth/catkin_generated'"
else
  . "/home/meka/mekabot/m3meka/ros/shm_led_mouth/catkin_generated/setup_cached.sh"
  exec "$@"
fi

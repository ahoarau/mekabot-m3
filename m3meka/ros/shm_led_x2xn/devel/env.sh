#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/meka/mekabot/m3meka/ros/shm_led_x2xn/devel', type 'exit' to leave"
  . "/home/meka/mekabot/m3meka/ros/shm_led_x2xn/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/meka/mekabot/m3meka/ros/shm_led_x2xn/devel'"
else
  . "/home/meka/mekabot/m3meka/ros/shm_led_x2xn/devel/setup.sh"
  exec "$@"
fi

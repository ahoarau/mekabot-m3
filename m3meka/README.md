## Welcome to m3meka.

m3 is open-source control software provided by Meka Robotics for its robots.
This is an update to match with recent configurations and needs at Ensta ParisTech.

This repository hosts all the Meka dependant components base on [m3core](https://github.com/ahoarau/m3core).
Please checkout https://github.com/ahoarau/mekabot for installation instructions.

>New features:
* Velocity control mode ( set_thetadot_deg in C++/Python API)
* Fixed differential butterworth instability bug
* Filters can be modified online (angle_df, torque_df etc in actuators config files)
* Supports Yamlcpp >0.5 (new API)
* Right and left hands are now part of the bot interface (C++/Python):
    * You can control them in ros_control (using https://github.com/ahoarau/m3ros_control)
    * In the Python API: bot.set_theta_deg("right_hand",[0,0,0,0,0])
* New Python API functions: get_ik, get_fk
* Various minor bug fixes


> Maintainer : Antoine Hoarau (hoarau.robotics@gmail.com)

[![Build Status](https://travis-ci.org/ahoarau/m3meka.svg?branch=master)](https://travis-ci.org/ahoarau/m3meka)

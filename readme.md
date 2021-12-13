# Playground

> UR5 Collaborative Robot Arm + Robotiq 85 2 finger gripper = <3

## Requirements

- A full desktop installation of [ROS1](http://wiki.ros.org) (tested on [noetic](http://wiki.ros.org/noetic)). You can install it using:
  - The [official installation](http://wiki.ros.org/noetic/Installation) method (only with Ubuntu 20.04)
  - The [robostack](https://robostack.github.io) project (Linux, macOS & Windows)

## Setup

Clone this repository:

```
git clone https://github.com/ros-unitn/playground --recurse-submodules
```

Install all [rosdep](https://www.google.com/search?client=safari&rls=en&q=rosdep&ie=UTF-8&oe=UTF-8) dependencies:

```
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
```

And build it!

```
make
```

## License

Copyright (C) 2021 Filippo Rossi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

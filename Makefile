# Copyright (C) 2021 Filippo Rossi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

current_dir = $(shell pwd)
home = ${HOME}

ifeq ($(OS),Windows_NT)
	operating_system := Windows
else
	operating_system := $(shell uname)
endif

ifeq ($(operating_system),Darwin)
	lib_ext := dylib
else
	lib_ext := so
endif

all: build

build:
	catkin_make	

%.$(lib_ext): devel
	cp $(current_dir)/devel/lib/$(@F) $@

$(home)/.gazebo/models:
	mkdir -p $(home)/.gazebo/models

%.model: $(home)/.gazebo/models
	cp -R $(current_dir)/models/$(@F) $@
libraries: $(home)/.ros/libgazebo_mimic_joint_plugin.$(lib_ext) $(home)/.ros/libx_linker.$(lib_ext)

models: $(home)/.gazebo/models/kinect.model $(home)/.gazebo/models/ur5_base.model $(home)/.gazebo/models/cafe_table.model $(home)/.gazebo/models/X2-Y2-Z2.model

run: devel libraries models
	roslaunch x_robot project.launch

clean:
	rm -rf $(home)/.ros/*.$(lib_ext)
	rm -rf $(home)/.gazebo/models/*.model

.PHONY: setup run clean libraries models build

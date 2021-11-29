current_dir = $(shell pwd)
home = ${HOME}

SHELL := /bin/bash

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

all: run

devel:
	catkin_make

%.$(lib_ext): devel
	cp $(current_dir)/devel/lib/$(@F) $@

%.model:
	cp -R $(current_dir)/models/$(@F) $@

setup: devel 
	source $(current_dir)/devel/setup.bash

libraries: $(home)/.ros/libgazebo_mimic_joint_plugin.$(lib_ext)

models: $(home)/.gazebo/models/kinect.model $(home)/.gazebo/models/ur5_base.model

run: setup libraries models
	roslaunch f_robot project.launch

clean:
	rm -rf $(home)/.ros/*.$(lib_ext)
	rm -rf $(home)/.gazebo/models/*.model

.PHONY: setup run clean libraries models
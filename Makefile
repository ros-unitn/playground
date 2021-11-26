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

setup: devel 
	source $(current_dir)/devel/setup.bash

run: setup $(home)/.ros/libgazebo_mimic_joint_plugin.$(lib_ext)
	roslaunch f_robot project.launch

clean:
	rm $(home)/.ros/*.$(lib_ext)

.PHONY: install clean
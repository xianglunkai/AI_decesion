#!/bin/bash

read -p "Please select build platform:[T3 x86_64]" platform
case $platform in
	T3)
	echo " build platform on T3"
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.cmake -S . -B ./build
	cd $(pwd)/build
	make
	;;
	x86_64)
	echo " build platform on x86_64"
	cd build
	cmake -S . -B ./build
	cd $(pwd)/build
	make
	;;
esac

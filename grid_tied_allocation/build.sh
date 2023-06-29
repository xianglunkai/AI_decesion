#!/bin/bash

read -p "Please select build platform:[T3 x64]" platform
case $platform in
	T3)
	echo " build platform on T3"
	rm -rf build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain.cmake -S . -B ./build
	cd $(pwd)/build
	make
	;;
	x64)
	echo " build platform on x64"
	rm -rf build
	cmake -S . -B ./build
	cd $(pwd)/build
	make
	;;
esac

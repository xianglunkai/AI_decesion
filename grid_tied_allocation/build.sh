#!/bin/bash

read -p "Please select build platform:[arm7.5 arm4.8 x64]" platform
case $platform in
	arm7.5)
	echo " build platform on arm7.5"
	rm -rf build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/armgcc7.5_toolchain.cmake -S . -B ./build
	cd $(pwd)/build
	make
	;;
	arm4.8)
	echo " build platform on arm4.8"
	rm -rf build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/armgcc4.8_toolchain.cmake -S . -B ./build
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

#!/bin/bash

read -p "Please select build platform:[arm7.5 arm4.8 x64]" platform
case $platform in
	arm7.5)
	echo " build platform on arm7.5"
	rm -rf build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/armgcc7.5_toolchain.cmake cmake -DCMAKE_INSTALL_PREFIX=./lib/grid_tied_allocation -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=ON -S . -B ./build
	cd $(pwd)/build
	make && make install
	make upload
	;;
	arm4.8)
	echo " build platform on arm4.8"
	rm -rf build
	cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/armgcc4.8_toolchain.cmake cmake -DCMAKE_INSTALL_PREFIX=./lib/grid_tied_allocation -DBUILD_SHARED_LIBS=OFF  -DBUILD_TESTING=ON -S . -B ./build
	cd $(pwd)/build
	make && make install
	;;
	x64)
	echo " build platform on x64"
	rm -rf build
	cmake -DCMAKE_INSTALL_PREFIX=./lib/grid_tied_allocation -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=ON -S . -B ./build
	cd $(pwd)/build
	make && make install
	;;
esac

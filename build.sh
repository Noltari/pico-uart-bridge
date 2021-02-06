#!/bin/sh

BUILD_DIR=build
PICO_SDK_DIR=pico-sdk

main () {
	local cur_dir=$PWD

	if [ ! -d "$PICO_SDK_DIR/.git" ]; then
		git submodule update --init --recursive
	fi

	mkdir -p $BUILD_DIR
	cd $BUILD_DIR
	cmake ../
	make

	cd $cur_dir
}

main $@

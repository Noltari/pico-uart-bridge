#!/bin/sh

BUILD_DIR=build

main () {
	local cur_dir=$PWD

	mkdir -p $BUILD_DIR
	cd $BUILD_DIR
	cmake ..
	make
	cd $cur_dir
}

main $@

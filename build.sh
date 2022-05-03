#!/bin/bash

BASE_DIR="$(dirname ${BASH_SOURCE[0]})"
BUILD_DIR=$BASE_DIR/build
PICO_SDK_DIR=$BASE_DIR/pico-sdk

main() {
	if [ ! -d "$PICO_SDK_DIR/.git" ]; then
		git submodule sync --recursive
		git submodule update --init --recursive
	fi

	cmake -B $BUILD_DIR -S $BASE_DIR
	make -C $BUILD_DIR
}

main $@

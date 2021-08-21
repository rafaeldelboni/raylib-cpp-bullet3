# Lists phony targets for Makefile
.PHONY: setup build

setup:
	cmake -B build

build:
	make --no-print-directory -C build

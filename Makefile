.SUFFIXES:

.PHONY: all
all: build/client

.PHONY: run
run: build/client
	build/client

.PHONY: clean
clean:
	-rm -rf build

build/client: build/Makefile
	(cd build; make -j)

build/Makefile: CMakeLists.txt build
	(cd build; cmake ..)

build:
	mkdir build

.SUFFIXES:

.PHONY: all
all: build/client

.PHONY: run
run: build/client
	build/client

.PHONY: test
test: build/client
	build/test_client

.PHONY: clean
clean:
	-rm -rf build

.PHONY: format
format:
	clang-format --style=file -i src/**.cpp src/**.hpp

build/client: build/Makefile
	(cd build; make -j)

build/Makefile: CMakeLists.txt build
	(cd build; cmake ..)

build:
	mkdir build

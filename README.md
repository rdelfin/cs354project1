# Getting Started

## Build

Assume the present working directory (PWD) is under the root directory of this
project. The following commands will create a directory named as "build" and
build the starter code within this directory.

```
mkdir build
cd build
cmake ..
make -j8
```

When finished, the binary called ray will be placed at build/bin/ directory.

You can change build to any name you like, although "build" is the most
commonly used one.

NOTE: It is recommended to use this build scheme (called out-of-source build).
Besides the advantages listed [here]
(http://voices.canonical.com/jussi.pakkanen/2013/04/16/why-you-should-consider-using-separate-build-directories/),
a more practical reason is we did not test in-source building at all.

## Dependencies

The starter code depends on FLTK and, certainly, the OpenGL to run.

UTCS UNIX hosts already have them installed. For your own system you may need
to install them through the corresponding package manager. If you are using OS
X, [Homebrew](http://brew.sh/) is recommended.

## How to add your file

In most cases you do not need to modify the build system manually to add new
files. The build system will automatically add source files under the
following directories to your project.

	src/
	src/fileio/
	src/parser/
	src/scene/
	src/SceneObjects
	src/ui/

However, since EVERY source file in these directories will be added to the
project to build, it is not a good practice to add random source files, say a
"Hello World" program, under these directories also, which will cause linking
errors due to multiple main functions.

On the other hand, all files out of these directories will NOT be added to the
project automatically. For example, if you created a directory called
src/kdtree/, and added your kd-tree implementation under this directory, these
kd-tree source files would NOT be built unless you modify the CMakeLists.txt
correspondingly.

## Tools that help you read the code

This starter code includes many features and lots of files, and usually it is
troublesome to jump over these files. For these cases, a source indexer helps
you a lot.

[cscope](http://cscope.sourceforge.net/) and [rtags](http://www.rtags.net/)
are two recommended ones. Cscope is included in most distributions, while
rtags requires build from source. However rtags understands C++ precisely
because it uses the Clang compiler fronted.

## Some notes about the compiler

On 3rd floor lab machines, the g++-5.2.0 will be used (located at
/lusr/opt/gcc-5.2.0/bin/g++). Otherwise the default C++ compiler is used.

C++11 is enabled by default.

## Some notes about glm

We disabled the member function 'length()' for all vector types to avoid
confusion, which returns the number of elements in a vector object, rather than
the norm of the vector. If you really want to get the number of elements in
the vector object, use 'size()' instead.

The build system will clone the 0.9.7.6 version from the official repository
if glm does not present. Otherwise, the glm installed by the system (may be
older) will be used.

# Acknowledgement 

This starter code is modified based on the ray tracing project of CS 384G.

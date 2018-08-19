# pcl-for-android

Bash scripts for cross compiling PCL ( https://github.com/PointCloudLibrary/pcl ) and its dependencies for Android.

## Install Requirements

```
$ sudo apt-get install git make cmake
```
## Additional Changes
a) Update the CMakeLists.txt file of "search" module. So that it looks like this 
```
set(srcs
        src/search.cpp
        src/kdtree.cpp
	    ../common/src/projection_matrix.cpp
        src/brute_force.cpp
        src/organized.cpp
        src/octree.cpp
        )
```

b) PCL's MovingLeastSquares algorithm fails on Android (core dumped). To make it work, turn off Eigen Vectorization.
1. Add the following lines in main CMakeLists.txt after ```find_package(Eigen REQUIRED)```


```
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT 
                      -DEIGEN_DONT_VECTORIZE) 
add_definitions(-DEIGEN_DONT_ALIGN)
```

2. Add the following lines in pcl_config.h.in after line 14/15. 
```
#define EIGEN_DONT_VECTORIZE 1 
#define EIGEN_DONT_ALIGN 1 
```


## Cross-compilation

```
$ git clone https://github.com/bashbug/pcl-for-android.git
$ export ANDROID_NDK=PATH_TO_YOUR_LOCAL_ANDROID_NDK_FOLDER
$ ./download-setup.sh
$ ./pcl-build-for-android.sh
```
## Android.mk 

I have included a sample jni folder including Android.mk & Application.mk. Update the paths accordingly. Tested with Android-Ndk-14b. 

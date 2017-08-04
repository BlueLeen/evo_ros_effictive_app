#!/bin/bash
if [ "$1" = "release" ]; then
    source env/arm/ros_evolver/install_isolated/setup.bash
    catkin_make_isolated --install  --install-space ros_app -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=${PWD}/env/arm/tools/rostoolchain.cmake
elif [ "$1" = "arm" ]; then    
catkin_make_isolated --install  --install-space ros_app -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DEVODEBUG=hello -DCMAKE_TOOLCHAIN_FILE=${PWD}/env/arm/tools/rostoolchain.cmake
elif [ "$1" = "x86" ]; then 
source /home/su/Workspace/ros_catkin_ws_full/install_isolated/setup.bash
catkin_make_isolated --install  --install-space ros_app -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug -DEVODEBUG=hello -DX86SWITCH=ON
else 
    source env/android/ros_evolver/ros_evolver/setup.bash
    if [ "$1" = "pkg" ]; then
        catkin_make_isolated --install  --install-space ros_app --pkg "$2" -DANDROID=1 -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug -DEVODEBUG=hello -DANDROIDSWITCH=on -DCMAKE_TOOLCHAIN_FILE=${PWD}/env/android/tools/rostoolchain.cmake
    else
        catkin_make_isolated --install  --install-space ros_app -DANDROID=1 -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Debug -DEVODEBUG=hello -DANDROIDSWITCH=on -DCMAKE_TOOLCHAIN_FILE=${PWD}/env/android/tools/rostoolchain.cmake
    fi
    source env/android/tools/elf_cleaner.sh ros_app
fi
 


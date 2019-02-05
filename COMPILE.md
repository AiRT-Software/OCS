# AIRT Project

This file explains how to build the On-board Control System binaries.

Hardware
--------
This project has been developed for the following platform:

-Up Board2 (https://up-board.org/upsquared) running Ubilinux
-Poxyz positioning system, with a 4-tag configuration, for acquiring orientation (https://www.pozyx.io/)
-Intel Realsense 3D camera. We support both ZR-300 and D415
-We used an external USB 3 Wifi adaptor to extend the range
-Gimbal
-ZCam E1 (4K professional recording camera)

Our project used a number of purpose-built components. Your hardware will be different, thus requiring developing new code  for interfacing with your specific hardware. The non-standard components are:
* The positioning system is connected to a hub in charge of computing the position of the center of the drone, and its heading. The hub feeds this data to the FCS (a standard Pixhawk) as if it were a GPS/compass, and to the OCS.
* In our project, the drone manufacturer developed a multiplexer module for accepting commands from both the manual RC and the OCS. The protocol is a simplified version of MAVLink.
* Finally, our gimbal also used a multiplexer for accepting commands both via radio and via the OCS. The multiplexer provided a small set of commands. You will have to adapt the system to your gimbal specific protocol.

Prerequisites
-------------
Install the system dependencies (see install-deps-up2.sh)
Download the library dependencies as explained in external-libs/readme.txt
Execute the compile.sh file in the root folder (it compiles the zeromq and restcpp libs).


For building the binaries with CMake
------------------------------------

1. Change to the trunk directory (where this file is located)
2. mkdir _build
3. cd _build
4. cmake ..
5. make install

Alternative builds
------------------

For building the project for *testing*, replace the step 4 above with:

cmake -DBUILD_TESTS=ON ..

For building the project in *debug*, replace the step 4 above with:

cmake -DCMAKE_BUILD_TYPE=Debug ..

The available build types in CMake are: Release, Debug, MinSizeRel and RelWithDebInfo.


For building the documentation
------------------------------

In the trunk folder, just run:

doxygen

The documentation in html form will be written to the folder trunk/doxygen



For building the deb file for installation in the production machine
--------------------------------------------------------------------
1. Change to the trunk directory (where this file is located)
2. mkdir _build
3. cd _build
4. Edit the file trunk/jiminy/airt_unity/Assets/Connection/Scripts/MessageCodes.cs 
   for updating the version in the enum AIRTVersion. The version defined in this 
   file will be used everywhere (in atreyu, jiminy and the version number in the 
   dpkg file).
4. cmake -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release ..
5. make package
6. Rename the generated package from airt-project_X.X.X-_amd64.deb to
   airt-project.bytes and move it into 
   trunk/jiminy/airt_unity/Assets/Resources/Update


==================================
mhls_utias_dataset
==================================

This is a ROS package to systematically read sensor msgs from the UTIAS-dataset (UTIAS MRCLAM robots dataset) and perform Moving horizon least squares on it for cooperative localization using inter-robot measurements.

==================================
Pre-requisites
==================================
1. Make sure you have ros indigo installed
2. Make sure you have the following package installed and the message files provided in this package built.

    https://github.com/aamirahmad/utiasdata_to_rosbags
    
3. Makse sure you have latest versions of the following libraries installed

   1. boost
   2. eigen
   3. g2o
   
4. you might have to add the location of installed g2o libraries in the CMakeLists.txt of this package

==================================
Instructions to use this package.
==================================

1. catkin_make to build the package
2. inspect the launch files, the names are self-explanatory
3. run one of the launch files. Below is an example:

    roslaunch mhls_utias_dataset fourRobotLocalization.launch MY_ID:=1 WINDOW_SIZE:=10 DECAY_LAMBDA:=10
  
4. In summary, all you provide as aruments s your robot id on which the estimation should run, window size of the moving horizon, decay lambda (see the publication) and the launch file which specifically says how many robots you want in the team.



==================================
Dataset download
==================================

See here: https://github.com/aamirahmad/utiasdata_to_rosbags

LICENSE: Please note that this package is released under the GNU GPLv3 License. 

==================================
Citation
==================================
To-be added



# cartpole_dynamic_goal
Simple example of how to add a new task to MJPC

The files on the cartpole_dynamic_goal folder are modifications of the original cartpole task in the MJPC folder mujoco_mpc/mjpc/tasks/cartpole

To build the CMakeLists.txt in the mujoco_mpc/tasks folder both need to be modified. 

 COMMAND ${CMAKE_COMMAND} -E copy
          ${dm_control_SOURCE_DIR}/dm_control/suite/cartpole.xml
          ${CMAKE_CURRENT_BINARY_DIR}/cartpole_dynamic_goal/cartpole.xml
 COMMAND patch -o ${CMAKE_CURRENT_BINARY_DIR}/cartpole_dynamic_goal/cartpole_modified.xml
          ${CMAKE_CURRENT_BINARY_DIR}/cartpole_dynamic_goal/cartpole.xml
          <${CMAKE_CURRENT_SOURCE_DIR}/cartpole_dynamic_goal/cartpole.xml.patch

and also the tasks.cc file,

std::make_shared<cartpole_dynamic_goal::CartpoleDynamicGoal>(),

And also the CMakeList.txt in the mujoco_mpc/mjpc folder needs modifying with,

tasks/cartpole_dynamic_goal/cartpole.cc   # <-- ADD THIS LINE
tasks/cartpole_dynamic_goal/cartpole.h

The build can then be done using clang, with

~/Python_Projects/mujoco_mpc/build$ cmake ..
~/Python_Projects/mujoco_mpc/build$ cmake --build .

Finally, launch mjpc using,

~/Python_Projects/mujoco_mpc/build$ ~/Python_Projects/mujoco_mpc/build/bin/mjpc 

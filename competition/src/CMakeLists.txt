set(ROBOT_STATE common/robotstate.h common/robotstate.cpp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

rosbuild_add_executable(statemachine statemachine.cpp ${ROBOT_STATE})
rosbuild_add_executable(explore explore.cpp ${ROBOT_STATE})
rosbuild_add_executable(ballcatcher ballcatcher.cpp ${ROBOT_STATE})

rosbuild_add_executable(laserposition laserposition.cpp)
rosbuild_add_executable(usbCom_Server usbCom.cpp)
rosbuild_add_executable(usbCom_client usbCom_client.cpp)

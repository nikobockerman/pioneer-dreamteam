set(ROBOT_STATE common/robotstate.h common/robotstate.cpp)
set(FUNCTIONS common/functions.h common/functions.cpp)

SET(DEFAULT_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "${DEFAULT_CXX_FLAGS} -std=c++0x")

rosbuild_add_executable(statemachine statemachine.cpp ${ROBOT_STATE})
rosbuild_add_executable(explore explore.cpp ${ROBOT_STATE})
rosbuild_add_executable(ballcatcher ballcatcher.cpp ${ROBOT_STATE} ${FUNCTIONS})
rosbuild_add_executable(laserposition laserposition.cpp)
rosbuild_add_executable(usbCom_Server usbCom.cpp)
rosbuild_add_executable(usbCom_client usbCom_client.cpp)
rosbuild_add_executable(ballPublisher ballPublisher.cpp ${FUNCTIONS})

rosbuild_add_executable(vision robot_vision.cpp)
set_target_properties(vision PROPERTIES COMPILE_FLAGS "${DEFAULT_CXX_FLAGS} -std=c++98")

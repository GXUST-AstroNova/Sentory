target_sources(${CMAKE_PROJECT_NAME} PRIVATE Gimbal.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR})

option(Gimbal_Pid_Debug "Enable the gimbal PID step debugger" ON)
if(Gimbal_Pid_Debug)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE Gimbal_Pid_Debug)
endif()

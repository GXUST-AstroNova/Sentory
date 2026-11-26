add_subdirectory(Btb)
add_subdirectory(Chassis)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE BoardConfig/BootConfig.cpp BoardConfig/BoardConfigCodec.cpp vofa.cpp PowerLimit/PowerLimit.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

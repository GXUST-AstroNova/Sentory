target_sources(${CMAKE_PROJECT_NAME} PRIVATE Application.cpp ChassisApp.cpp GimbalApp.cpp)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})

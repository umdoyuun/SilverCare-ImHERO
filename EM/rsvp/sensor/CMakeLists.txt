cmake_minimum_required(VERSION 3.10)
project(rsvp_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
)

# Header files
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# Link wiringPi library
target_link_libraries(${PROJECT_NAME} 
	wiringPi
	${CURL_LIBRARIES}
	nlohmann_json::nlohmann_json
	OpenSSL::Crypto
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CURL_INCLUDE_DIRS}
)

# Add clean target
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/Makefile
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/cmake_install.cmake
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/${PROJECT_NAME}
)

# Add test directory if tests exist
add_subdirectory(test)

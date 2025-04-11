# Test source files
file(GLOB TEST_SOURCES
    "*.cpp"
)

# Main project source files (excluding main.cpp)
file(GLOB_RECURSE PROJECT_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# Remove main.cpp from project sources
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

add_executable(test_runner ${TEST_SOURCES} ${PROJECT_SOURCES})
target_link_libraries(test_runner 
    wiringPi
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
	OpenSSL::Crypto
)
target_include_directories(test_runner PRIVATE ${CMAKE_SOURCE_DIR}/include)

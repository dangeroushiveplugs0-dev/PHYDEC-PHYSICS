cmake_minimum_required(VERSION 3.20)
project(phydec_external_consumer LANGUAGES CXX)

find_package(PHYDEC CONFIG REQUIRED)

add_executable(phydec_consumer main.cpp)
target_link_libraries(phydec_consumer PRIVATE PHYDEC::Core)
target_compile_features(phydec_consumer PRIVATE cxx_std_20)

enable_testing()
add_test(NAME phydec_consumer_runs COMMAND phydec_consumer)

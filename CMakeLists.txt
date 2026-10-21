cmake_minimum_required(VERSION 3.20)
project(PHYDEC_PHYSICS VERSION 0.1.0 LANGUAGES CXX)

option(PHYDEC_BUILD_TESTS "Build PHYDEC automated tests" ON)

add_subdirectory(engine)

if(PHYDEC_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

add_library(phydec_core
  src/secondary_motion.cpp
)
add_library(PHYDEC::Core ALIAS phydec_core)

target_compile_features(phydec_core PUBLIC cxx_std_20)
target_include_directories(phydec_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(MSVC)
  target_compile_options(phydec_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(phydec_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

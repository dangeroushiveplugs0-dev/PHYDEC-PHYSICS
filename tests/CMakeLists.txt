add_executable(phydec_core_tests
  secondary_motion_tests.cpp
)
target_link_libraries(phydec_core_tests PRIVATE PHYDEC::Core)
target_compile_features(phydec_core_tests PRIVATE cxx_std_20)
add_test(NAME phydec_core_tests COMMAND phydec_core_tests)

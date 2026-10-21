add_library(phydec_core
  src/secondary_motion.cpp
)
add_library(PHYDEC::Core ALIAS phydec_core)
set_target_properties(phydec_core PROPERTIES
  EXPORT_NAME Core
  CXX_EXTENSIONS OFF
)
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

if(PHYDEC_INSTALL)
  include(GNUInstallDirs)
  include(CMakePackageConfigHelpers)

  install(TARGETS phydec_core
    EXPORT PHYDECTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  )
  install(EXPORT PHYDECTargets
    FILE PHYDECTargets.cmake
    NAMESPACE PHYDEC::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PHYDEC
  )
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/PHYDECConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/PHYDECConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PHYDEC
  )
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/PHYDECConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/PHYDEC
  )
endif()

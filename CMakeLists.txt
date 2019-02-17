# @author Munzir Zafar
# @date Dec 12, 2018

cmake_minimum_required(VERSION 2.6)

install(DIRECTORY "betaConvergence"
  DESTINATION /usr/local/share/krang
  FILES_MATCHING
  PATTERN "*.txt"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

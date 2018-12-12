# @author Munzir Zafar
# @date Dec 12, 2018

cmake_minimum_required(VERSION 2.6)

install(DIRECTORY "betaConvergence"
  DESTINATION /usr/local/share/krang
  FILES_MATCHING
  PATTERN "*.txt"
  PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)



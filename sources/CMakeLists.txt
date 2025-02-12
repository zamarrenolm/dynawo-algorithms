# Copyright (c) 2015-2021, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source suite
# of simulation tools for power systems.

set(DYN_ALGO_SOURCES
  main.cpp
  )

set(DYN_ALGO_INCLUDE_HEADERS
  config_algorithms.h
  gitversion_algorithms.h
  )

install(FILES ${DYN_ALGO_INCLUDE_HEADERS} DESTINATION ${INCLUDEDIR_NAME})

add_executable(dynawoAlgorithms ${DYN_ALGO_SOURCES})
target_link_libraries(dynawoAlgorithms
  dynawo_algorithms_Launcher
  $<$<NOT:$<BOOL:${USE_LEGACY_IIDM}>>:LibXml2::LibXml2>
  )

install(PROGRAMS execDynawoAlgorithms.sh DESTINATION ${BINDIR_NAME})

set_target_properties(dynawoAlgorithms PROPERTIES VERSION ${DYNAWO_ALGORITHMS_VERSION_STRING})
install(TARGETS dynawoAlgorithms DESTINATION ${BINDIR_NAME})

# explicitly say that the executable depends on custom target
add_dependencies(dynawoAlgorithms revisiontag)

#obtain git revision
# a custom target that is always built
add_custom_target(revisiontag ALL)

# creates gitversion_algorithms.h using cmake script
add_custom_command(TARGET revisiontag
  COMMAND ${CMAKE_COMMAND}
    -DOUTPUT_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DGIT_SOURCE_DIR=${DYNAWO_ALGORITHMS_HOME}
    -P ${DYNAWO_ALGORITHMS_CMAKE_MODULE_PATH}/getgitheadrevision.cmake
  )

add_subdirectory(API)
add_subdirectory(Common)
add_subdirectory(Launcher)

if(BUILD_TESTS OR BUILD_TESTS_COVERAGE)
  add_subdirectory(Test)
  add_subdirectory(API/aggregatedResults/test)
  add_subdirectory(API/multipleJobs/test)
  add_subdirectory(Common/test)
  add_subdirectory(Launcher/test)
endif()

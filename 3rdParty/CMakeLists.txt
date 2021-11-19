# Copyright (c) 2021, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# Minimum required (for ExternalProject_Add)
cmake_minimum_required(VERSION 3.9.6 FATAL_ERROR)

project("dynawo-algorithms-3rd"
LANGUAGES C CXX
)

include(ExternalProject)

set(DOWNLOAD_DIR ${CMAKE_INSTALL_PREFIX}/src CACHE PATH "Directory where 3rd parties are downloaded.")
set(TMP_DIR ${CMAKE_INSTALL_PREFIX}/tmp CACHE PATH "Directory where 3rd parties temporary files are created.")

set(mpich_url http://www.mpich.org/static/downloads/3.4.2/mpich-3.4.2.tar.gz)
set(MPI_HOME ${CMAKE_INSTALL_PREFIX}/mpich)
find_package(MPI QUIET)
if(MPI_FOUND)
  add_custom_target(mpich)
  set(MPI_HOME ${CMAKE_INSTALL_PREFIX}/mpich)
else()
  ExternalProject_Add(mpich
      INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}/mpich
      DOWNLOAD_DIR        ${CMAKE_CURRENT_SOURCE_DIR}/mpich
      TMP_DIR             ${TMP_DIR}
      STAMP_DIR           ${DOWNLOAD_DIR}/mpich-stamp
      SOURCE_DIR          ${DOWNLOAD_DIR}/mpich
      URL                 ${mpich_url}
      BUILD_IN_SOURCE     1

      CONFIGURE_COMMAND   <SOURCE_DIR>/configure
                          --prefix=<INSTALL_DIR>
                          "CC=${CMAKE_C_COMPILER}"
                          "CXX=${CMAKE_CXX_COMPILER}"
                          "CFLAGS=-m64 -O3 -DNDEBUG"
                          "CXXFLAGS=-m64 -O3 -DNDEBUG"
                          "FCFLAGS=-m64 -O3 -DNDEBUG"
                          "LDFLAGS=-m64 -O3 -DNDEBUG"
                          --with-device=ch3:nemesis
                          --disable-fortran
                          --enable-fast=all,O3,ndebug
  )
  ExternalProject_Get_Property(mpich install_dir)
  set(MPI_HOME ${install_dir})
endif()

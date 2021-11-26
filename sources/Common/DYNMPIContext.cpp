//
// Copyright (c) 2021, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source suite
// of simulation tools for power systems.
//

#include "DYNMPIContext.h"

#include <iostream>
#include <numeric>

namespace DYNAlgorithms {
namespace mpi {

Context&
Context::instance() {
  static Context context;
  return context;
}

Context::Context() {
  int ret = MPI_Init(NULL, NULL);
  if (ret != MPI_SUCCESS) {
    std::cerr << "Error initialization MPI" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  ret = MPI_Comm_size(MPI_COMM_WORLD, &nbProcs_);
  if (ret != MPI_SUCCESS) {
    std::cerr << "Error acquiring number of MPI process" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  ret = MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
  if (ret != MPI_SUCCESS) {
    std::cerr << "Error initialization MPI" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

Context::~Context() {
  MPI_Finalize();
}

void
forEach(unsigned int iStart, unsigned int size, const std::function<void(unsigned int)>& func) {
  auto& context = Context::instance();
  for (unsigned int i = iStart; i < size; i++) {
    if (i % context.nbProcs() == context.rank()) {
      func(i);
    }
  }
}

template<>
void
Context::gather_impl(Tag<bool>, const bool& data, std::vector<bool>& recvData) const {
  std::vector<unsigned int> recvDataInt;
  gather(static_cast<unsigned int>(data), recvDataInt);
  if (isRootProc()) {
    recvData.assign(recvDataInt.begin(), recvDataInt.end());
  }
}

template<>
void
Context::gather_impl(Tag<std::vector<bool> >, const std::vector<bool>& data, std::vector<std::vector<bool> >& recvData) const {
  std::vector<std::vector<unsigned int> > recvDataInt;
  std::vector<unsigned int> dataInt(data.begin(), data.end());
  gather(dataInt, recvDataInt);
  if (isRootProc()) {
    recvData.resize(recvDataInt.size());
    for (unsigned int i = 0; i < recvDataInt.size(); i++) {
      recvData.at(i).assign(recvDataInt.at(i).begin(), recvDataInt.at(i).end());
    }
  }
}

template<>
void
Context::gather_impl(Tag<std::string>, const std::string& data, std::vector<std::string>& recvData) const {
  std::vector<unsigned char> dataStr(data.begin(), data.end());
  std::vector<std::vector<unsigned char> > ret;
  gather(dataStr, ret);
  if (isRootProc()) {
    recvData.resize(nbProcs_);
    for (unsigned int i = 0; i < ret.size(); i++) {
      recvData.at(i) = std::string(ret.at(i).begin(), ret.at(i).end());
    }
  }
}

template<>
void
Context::broadcast_impl(Tag<std::string>, std::string& data) const {
  std::vector<unsigned char> dataVect;
  if (isRootProc()) {
    dataVect.assign(data.begin(), data.end());
  }
  broadcast(dataVect);
  if (!isRootProc()) {
    data.assign(dataVect.begin(), dataVect.end());
  }
}

template<>
void
Context::broadcast_impl(Tag<bool>, bool& data) const {
  unsigned int dataInt = static_cast<unsigned int>(data);
  broadcast(dataInt);
  data = static_cast<bool>(dataInt);
}

template<>
void
Context::broadcast_impl(Tag<std::vector<bool> >, std::vector<bool>& data) const {
  std::vector<unsigned int> dataInt;
  if (isRootProc()) {
    dataInt.assign(data.begin(), data.end());
  }
  broadcast(dataInt);
  if (!isRootProc()) {
    std::transform(dataInt.begin(), dataInt.end(), std::back_inserter(data), [](unsigned int data) { return static_cast<bool>(data); });
  }
}

}  // namespace mpi

}  // namespace DYNAlgorithms

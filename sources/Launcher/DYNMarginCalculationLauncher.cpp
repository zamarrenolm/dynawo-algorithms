//
// Copyright (c) 2015-2021, RTE (http://www.rte-france.com)
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

/**
 * @file  MarginCalculationLauncher.cpp
 *
 * @brief Margin Calculation launcher: implementation of the algorithm and interaction with dynawo core
 *
 */

#include <iostream>
#include <cmath>
#include <ctime>
#include <iomanip>

#include <libzip/ZipFile.h>
#include <libzip/ZipFileFactory.h>
#include <libzip/ZipEntry.h>
#include <libzip/ZipInputStream.h>
#include <libzip/ZipOutputStream.h>

#include <DYNFileSystemUtils.h>
#include <DYNExecUtils.h>
#include <DYNSimulation.h>
#include <DYNSubModel.h>
#include <DYNTrace.h>
#include <DYNModel.h>
#include <DYNParameter.h>
#include <DYNModelMulti.h>
#include <JOBXmlImporter.h>
#include <JOBIterators.h>
#include <JOBJobsCollection.h>
#include <JOBJobEntry.h>
#include <JOBSimulationEntry.h>

#include "DYNMarginCalculationLauncher.h"
#include "DYNMultipleJobs.h"
#include "DYNMarginCalculation.h"
#include "DYNScenario.h"
#include "MacrosMessage.h"
#include "DYNScenarios.h"
#include "DYNAggrResXmlExporter.h"
#include "DYNMPIContext.h"

#include <unordered_set>

using multipleJobs::MultipleJobs;
using DYN::Trace;

namespace DYNAlgorithms {

static DYN::TraceStream TraceInfo(const std::string& tag = "") {
  return mpi::context().isRootProc() ? Trace::info(tag) : DYN::TraceStream();
}

void
MarginCalculationLauncher::createScenarioWorkingDir(const std::string& scenarioId, double variation) const {
  std::stringstream subDir;
  subDir << "step-" << variation << "/" << scenarioId;
  std::string workingDir = createAbsolutePath(subDir.str(), workingDirectory_);
  if (!exists(workingDir))
    create_directory(workingDir);
  else if (!is_directory(workingDir))
    throw DYNAlgorithmsError(DirectoryDoesNotExist, workingDir);
}

void
MarginCalculationLauncher::readTimes(const std::string& jobFileLoadIncrease, const std::string& jobFileScenario) {
  // job
  job::XmlImporter importer;
  boost::shared_ptr<job::JobsCollection> jobsCollection = importer.importFromFile(workingDirectory_ + "/" + jobFileLoadIncrease);
  //  implicit : only one job per file
  job::job_iterator jobIt = jobsCollection->begin();
  tLoadIncrease_ = (*jobIt)->getSimulationEntry()->getStopTime();

  jobsCollection = importer.importFromFile(workingDirectory_ + "/" + jobFileScenario);
  jobIt = jobsCollection->begin();
  tScenario_ = (*jobIt)->getSimulationEntry()->getStopTime() - (*jobIt)->getSimulationEntry()->getStartTime();
}

void MarginCalculationLauncher::launchLoadIncreases(const boost::shared_ptr<LoadIncrease>& loadIncrease, const std::vector<unsigned int>& variations) {
  if (variations.empty()) {
    return;
  }

  // Launch Simulations
  inputs_.readInputs(workingDirectory_, loadIncrease->getJobsFile());
  std::vector<bool> successes;
  mpi::forEach(0, variations.size(), [this, &loadIncrease, &variations, &successes](unsigned int i){
    SimulationResult result;
    launchLoadIncrease(loadIncrease, variations.at(i), result);
    successes.push_back(result.getSuccess());
    exportResult(result);
  });
  // Sync sucesses
  std::vector<bool> allSuccesses = synchronizeSuccesses(successes);
  // Fill load increase status
  for (unsigned int i = 0; i < variations.size(); i++) {
    loadIncreaseStatus_.insert(std::make_pair(variations.at(i), LoadIncreaseStatus(allSuccesses.at(i))));
  }
  // log infos
  if (mpi::context().isRootProc()) {
    for (unsigned int variation : variations) {
      auto result = importResult(computeLoadIncreaseScenarioId(variation));
      TraceInfo(logTag_) << DYNAlgorithmsLog(LoadIncreaseEnd, variation, getStatusAsString(result.getStatus())) << Trace::endline;
    }
  }
}

auto MarginCalculationLauncher::computeScenarioSimulations(const std::vector<unsigned int>& variations,
    const std::vector<boost::shared_ptr<Scenario>>& scenarios) const -> std::vector<ScenarioSimulation> {
  // Compute number of simulation to launch : must be a multiple of nb procs and the first variation must be processed completly
  auto& context = mpi::context();
  unsigned int nbSimulations = 0;
  auto firstVariation = variations.front();
  for (unsigned int i = 0; i < scenarios.size(); i++) {
    if (scenarioStatus_.count(firstVariation) > 0 && scenarioStatus_.at(firstVariation).count(i) > 0) continue;
    nbSimulations++;
  }
  nbSimulations = (nbSimulations / context.nbProcs() + 1) * context.nbProcs();

  std::vector<ScenarioSimulation> simulations;
  for (unsigned int variation : variations) {
    if (loadIncreaseStatus_.count(variation) == 0 || !loadIncreaseStatus_.at(variation).success) continue;
    for (unsigned int i = 0; i < scenarios.size(); i++) {
      if (scenarioStatus_.count(firstVariation) > 0 && scenarioStatus_.at(firstVariation).count(i) > 0) continue;
      simulations.emplace_back(variation, i);
      if (simulations.size() >= nbSimulations) {
        return simulations;
      }
    }
  }

  return simulations;
}

bool
MarginCalculationLauncher::IsScenarioPassedWithBetterVariation(const std::string& scenarioId, unsigned int variation) const {
  return scenarioVariationPassed_.count(scenarioId) > 0 && variation <= scenarioVariationPassed_.at(scenarioId);
}

std::vector<unsigned int>
MarginCalculationLauncher::performLoadIncreases(const std::vector<unsigned int>& variations) {
  const auto& marginCalculation = multipleJobs_->getMarginCalculation();
  const auto& loadIncrease = marginCalculation->getLoadIncrease();
  auto& generator = dynamic_cast<GlobalMarginSimulation*>(marginSimulation_.get())->variationGenerator();
  auto& context = mpi::context();

  auto firstVariation = variations.front();
  if (loadIncreaseStatus_.count(firstVariation) > 0 && loadIncreaseStatus_.at(firstVariation).success) {
    // highest variation of the list has already been processed
    // it might be counter-productive to perform load increases without testing this one so we select all the valid load increase variations
    return filterNOKVariations(variations);
  }

  // count how many of these variations have already been processed
  auto count = std::count_if(variations.begin(), variations.end(), [this](unsigned int variation){
    return loadIncreaseStatus_.count(variation) > 0;
  });
  auto newVariations = generator.generate(context.nbProcs() + count);
  std::vector<unsigned int> variationsLoadIncrease;
  for (unsigned int variation : newVariations) {
    if (loadIncreaseStatus_.count(variation) > 0) continue;
    variationsLoadIncrease.push_back(variation);
  }

  // launch load increase
  launchLoadIncreases(loadIncrease, variationsLoadIncrease);
  for (unsigned int variation : variationsLoadIncrease) {
    if (!loadIncreaseStatus_.at(variation).success) {
      generator.valid(variation, false);
    }
  }

  return filterNOKVariations(newVariations);
}

std::vector<unsigned int>
MarginCalculationLauncher::filterNOKVariations(const std::vector<unsigned int>& variations) const {
  std::vector<unsigned int> ret;
  for (unsigned int variation : variations) {
    if (loadIncreaseStatus_.count(variation) > 0 && loadIncreaseStatus_.at(variation).success) {
      ret.push_back(variation);
    }
  }
  return ret;
}

unsigned int
MarginCalculationLauncher::computeGlobalMargin() {
  const auto& marginCalculation = multipleJobs_->getMarginCalculation();
  const auto& scenarios = marginCalculation->getScenarios()->getScenarios();
  const auto& loadIncrease = marginCalculation->getLoadIncrease();
  auto& generator = dynamic_cast<GlobalMarginSimulation*>(marginSimulation_.get())->variationGenerator();
  auto& context = mpi::context();

#if _DEBUG_
  // variable to check that the algorithm doesn't loop indefinitely
  auto checkAlgoVar = generator.distance();
#endif
  while (generator.distance() > static_cast<unsigned int>(marginCalculation->getAccuracy())) {
    LoadIncreaseResult result;
    result.resize(scenarios.size());

    // Generate variations to process in load increase
    auto variations = generator.generate(context.nbProcs());
    variations = performLoadIncreases(variations);

    if (variations.empty()) {
      continue;
    }
    auto variation = variations.front();
    result.setLoadLevel(variation);

    // launch scenarios
    auto simulations = computeScenarioSimulations(variations, scenarios);
    launchScenarios(simulations, marginCalculation->getScenarios());

    // Update generators with failures
    for (const auto& simulation : simulations) {
      if (!scenarioStatus_.at(simulation.variation).at(simulation.scenarioIdx).success) {
        // If a scenario fails on a variation, the whole variation shall be considered non valid
        generator.valid(simulation.variation, false);
      }
    }

    // determine which scenarios were run for first variation and update
    std::unordered_set<std::string> runScenarios;
    for (const auto& simulation : simulations) {
      if (variation == simulation.variation) {
        const auto& scenarioId = scenarios.at(simulation.scenarioIdx)->getId();
        if (!IsScenarioPassedWithBetterVariation(scenarioId, simulation.variation)) {
          scenarioVariationPassed_[scenarioId] = simulation.variation;
        }
        runScenarios.insert(scenarioId);
      }
    }

    // process first variation result
    updateResults(result, variation, scenarios);
    auto statusFound = scenarioStatus_.find(variation);
    unsigned int nbSuccesses = 0;
    for (unsigned int i = 0; i < scenarios.size(); i++) {
      const auto& scenarioId = scenarios.at(i)->getId();
      auto uniqueId = SimulationResult::getUniqueScenarioId(scenarioId, variation);
      if (runScenarios.count(scenarioId) == 0) {
        // If scenario was not run for first variation, it means that it worked with previous variation: it is considered successed
        nbSuccesses++;
        TraceInfo(logTag_) << DYNAlgorithmsLog(ScenarioNotSimulated, uniqueId) << Trace::endline;
      } else {
        assert(statusFound != scenarioStatus_.end());  // if scenario was not filtered, it should have been run for first variation
        if (statusFound->second.at(i).success) {
          nbSuccesses++;
        }
        // Scenario end log already done in launch scenarios
      }
    }
    generator.valid(variation, nbSuccesses == scenarios.size());
    results_.push_back(result);

    TraceInfo(logTag_) << Trace::endline;
#if _DEBUG_
    assert(generator.distance() < checkAlgoVar);
    checkAlgoVar = generator.distance();
#endif
  }

  return generator.min();
}

void MarginCalculationLauncher::launchScenarios(const std::vector<ScenarioSimulation>& events, const boost::shared_ptr<Scenarios>& scenarios) {
  const auto& baseJobsFile = scenarios->getJobsFile();
  const auto& scenariosVect = scenarios->getScenarios();

  if (events.empty()) {
    return;
  }

  for (const auto& pair : events) {
    auto variation = pair.variation;
    auto iidmFile = generateIDMFileNameForVariation(variation);
    if (inputsByIIDM_.count(iidmFile) == 0) {
      inputsByIIDM_[iidmFile].readInputs(workingDirectory_, baseJobsFile, iidmFile);
    }
  }

  std::vector<bool> successes;
  mpi::forEach(0, events.size(), [this, &scenariosVect, &events, &successes](unsigned int i){
    auto variation = events.at(i).variation;
    auto iidmFile = generateIDMFileNameForVariation(variation);
    auto eventIdx = events.at(i).scenarioIdx;
    SimulationResult result;
    const auto& event = scenariosVect.at(eventIdx);
    createScenarioWorkingDir(event->getId(), variation);
    launchScenario(inputsByIIDM_.at(iidmFile), event, variation, result);
    successes.push_back(result.getSuccess());
    exportResult(result);
  });
  // Sync sucesses
  std::vector<bool> allSuccesses = synchronizeSuccesses(successes);
  for (unsigned int i = 0; i < events.size(); i++) {
    const auto& event = events.at(i);
    scenarioStatus_[event.variation][event.scenarioIdx].success = allSuccesses.at(i);
  }

  if (mpi::context().isRootProc()) {
    for (const auto& event : events) {
      const auto& scenarioId = scenariosVect.at(event.scenarioIdx)->getId();
      auto resultId = SimulationResult::getUniqueScenarioId(scenarioId, event.variation);
      auto result = importResult(resultId);
      TraceInfo(logTag_) << DYNAlgorithmsLog(ScenariosEnd, resultId, getStatusAsString(result.getStatus())) << Trace::endline;
    }
  }

  for (const auto& event : events) {
    auto variation = event.variation;
    auto iidmFile = generateIDMFileNameForVariation(variation);
    inputsByIIDM_.erase(iidmFile);  // remove iidm file used for scenario to save RAM
  }
}

void
MarginCalculationLauncher::updateResults(LoadIncreaseResult& result, unsigned int variation,
  const std::vector<boost::shared_ptr<DYNAlgorithms::Scenario>>& scenarios) const {
  auto loadIncreaseSimulationStatus = importResult(computeLoadIncreaseScenarioId(variation));
  result.setStatus(loadIncreaseSimulationStatus.getStatus());
  for (unsigned int i = 0; i < scenarios.size(); i++) {
    const auto& scenario = scenarios.at(i);
    auto resultId = SimulationResult::getUniqueScenarioId(scenario->getId(), variation);
    result.getResult(i) = importResult(resultId);
  }
}

void
MarginCalculationLauncher::launch() {
  assert(multipleJobs_);
  results_.clear();
  boost::shared_ptr<MarginCalculation> marginCalculation = multipleJobs_->getMarginCalculation();
  if (!marginCalculation) {
    throw DYNAlgorithmsError(MarginCalculationTaskNotFound);
  }
  const auto& loadIncrease = marginCalculation->getLoadIncrease();
  const auto& scenarios = marginCalculation->getScenarios();
  const auto& baseJobsFile = scenarios->getJobsFile();
  if (marginCalculation->getCalculationType() == MarginCalculation::GLOBAL_MARGIN) {
    marginSimulation_ = std::make_shared<GlobalMarginSimulation>(multipleJobs_);
  } else {
    // local margin
    marginSimulation_ = std::make_shared<LocalMarginSimulation>(multipleJobs_);
  }
  auto& context = mpi::context();

  // Retrieve from jobs file tLoadIncrease and tScenario
  readTimes(loadIncrease->getJobsFile(), baseJobsFile);

  // First load increases: we start with range [100 50]
  std::vector<unsigned int> variations{100};
  VariationGenerator generatorInit(50, 100, marginCalculation->getAccuracy());
  auto otherVariations = generatorInit.generate(context.nbProcs() - 1);
  variations.insert(variations.end(), otherVariations.begin(), otherVariations.end());
  launchLoadIncreases(loadIncrease, variations);
  for (unsigned int variation : variations) {
    // we valid the real generators
    if (!loadIncreaseStatus_.at(variation).success) {
      marginSimulation_->validVariationAll(variation, false);
    }
  }

  if (loadIncreaseStatus_.at(100).success) {
    // launch all scenarios
    const auto& scenariosVect = scenarios->getScenarios();
    auto events = computeScenarioSimulations(variations, scenariosVect);
    launchScenarios(events, scenarios);

    // Update with failures for global
    if (marginCalculation->getCalculationType() == MarginCalculation::GLOBAL_MARGIN) {
      for (const auto& simulation : events) {
        if (!scenarioStatus_.at(simulation.variation).at(simulation.scenarioIdx).success) {
          // If a scenario fails on a variation, the whole variation shall be considered non valid
          marginSimulation_->validVariationAll(simulation.variation, false);
        }
      }
    }

    // Check success for 100
    LoadIncreaseResult result100;
    result100.setLoadLevel(100);
    result100.resize(scenariosVect.size());
    updateResults(result100, 100, scenariosVect);
    results_.push_back(result100);
    TraceInfo(logTag_) << Trace::endline;

    const auto& status = scenarioStatus_.at(100);
    // we have the garantee here that every scenario was run
    assert(status.size() == scenariosVect.size());
    bool allSuccess = std::all_of(status.begin(), status.end(), [](const std::pair<unsigned int, LoadIncreaseStatus>& pair){
      return pair.second.success;
    });
    if (allSuccess) {
      TraceInfo(logTag_) << "============================================================ " << Trace::endline;
      TraceInfo(logTag_) << DYNAlgorithmsLog(GlobalMarginValue, 100) << Trace::endline;
      TraceInfo(logTag_) << "============================================================ " << Trace::endline;
      return;
    } else {
      marginSimulation_->validVariationAll(100, false);
    }
  }

  // compute margins
  if (marginCalculation->getCalculationType() == MarginCalculation::GLOBAL_MARGIN) {
    auto value = computeGlobalMargin();
    TraceInfo(logTag_) << "============================================================ " << Trace::endline;
    TraceInfo(logTag_) << DYNAlgorithmsLog(GlobalMarginValue, value) << Trace::endline;
    TraceInfo(logTag_) << "============================================================ " << Trace::endline;
  } else {
    assert(false);  // TODO(lecourtoisflo) to implement
  }
}

// void
// MarginCalculationLauncher::launch() {
//   assert(multipleJobs_);
//   results_.clear();
//   boost::shared_ptr<MarginCalculation> marginCalculation = multipleJobs_->getMarginCalculation();
//   if (!marginCalculation) {
//     throw DYNAlgorithmsError(MarginCalculationTaskNotFound);
//   }
//   const boost::shared_ptr<LoadIncrease>& loadIncrease = marginCalculation->getLoadIncrease();
//   const boost::shared_ptr<Scenarios>& scenarios = marginCalculation->getScenarios();
//   const std::string& baseJobsFile = scenarios->getJobsFile();
//   const std::vector<boost::shared_ptr<Scenario> >& events = scenarios->getScenarios();

//   // Retrieve from jobs file tLoadIncrease and tScenario
//   readTimes(loadIncrease->getJobsFile(), baseJobsFile);

//   std::queue< task_t > toRun;
//   std::vector<size_t> allEvents;
//   for (size_t i=0, iEnd = events.size(); i < iEnd ; i++)
//     allEvents.push_back(i);
//   toRun.push(task_t(100, 100, allEvents));

//   results_.push_back(LoadIncreaseResult());
//   size_t idx = results_.size() - 1;
//   results_[idx].resize(events.size());
//   results_[idx].setLoadLevel(100.);

//   // step one : launch the loadIncrease and then all events with 100% of the load increase
//   // if there is no crash => no need to go further
//   // We start with 100% as it is the most common result of margin calculations on real large cases
//   SimulationResult result100;
//   findOrLaunchLoadIncrease(loadIncrease, 100, marginCalculation->getAccuracy(), result100);
//   results_[idx].setStatus(result100.getStatus());
//   std::vector<double > maximumVariationPassing(events.size(), 0.);
//   if (result100.getSuccess()) {
//     findAllLevelsBetween(0., 100., marginCalculation->getAccuracy(), allEvents, toRun);
//     findOrLaunchScenarios(baseJobsFile, events, toRun, results_[idx]);

//     // analyze results
//     unsigned int nbSuccess = 0;
//     size_t id = 0;
//     for (std::vector<SimulationResult>::const_iterator it = results_[idx].begin(),
//         itEnd = results_[idx].end(); it != itEnd; ++it, ++id) {
//       Trace::info(logTag_) << DYNAlgorithmsLog(ScenariosEnd, it->getUniqueScenarioId(), getStatusAsString(it->getStatus())) << Trace::endline;
//       if (it->getStatus() == CONVERGENCE_STATUS) {  // event OK
//         nbSuccess++;
//         maximumVariationPassing[id] = 100;
//       }
//     }
//     if (nbSuccess == events.size()) {  // all events succeed
//       Trace::info(logTag_) << "============================================================ " << Trace::endline;
//       Trace::info(logTag_) << DYNAlgorithmsLog(GlobalMarginValue, 100.) << Trace::endline;
//       Trace::info(logTag_) << "============================================================ " << Trace::endline;
//       return;
//     }
//   }  // if the loadIncrease failed, nothing to do, the next algorithm will try to find the right load level
//   Trace::info(logTag_) << Trace::endline;

//   toRun = std::queue< task_t >();

//   results_.push_back(LoadIncreaseResult());
//   idx = results_.size() - 1;
//   results_[idx].resize(events.size());
//   results_[idx].setLoadLevel(0.);
//   // step two : launch the loadIncrease and then all events with 0% of the load increase
//   // if one event crash => no need to go further
//   SimulationResult result0;
//   findOrLaunchLoadIncrease(loadIncrease, 0, marginCalculation->getAccuracy(), result0);
//   results_[idx].setStatus(result0.getStatus());
//   if (result0.getSuccess()) {
//     toRun = std::queue< task_t >();
//     std::vector<size_t> eventsIds;
//     for (size_t i = 0, iEnd = events.size(); i < iEnd ; i++) {
//       if (0. > maximumVariationPassing[i] || DYN::doubleEquals(0., maximumVariationPassing[i])) {
//         eventsIds.push_back(i);
//       } else {
//         Trace::info(logTag_) << DYNAlgorithmsLog(ScenarioNotSimulated, events[i]->getId()) << Trace::endline;
//         results_[idx].getResult(i).setScenarioId(events[i]->getId());
//         results_[idx].getResult(i).setVariation(0.);
//         results_[idx].getResult(i).setSuccess(true);
//         results_[idx].getResult(i).setStatus(CONVERGENCE_STATUS);
//       }
//     }
//     toRun.push(task_t(0., 0., eventsIds));
//     findOrLaunchScenarios(baseJobsFile, events, toRun, results_[idx]);
//   } else {
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//     Trace::info(logTag_) << DYNAlgorithmsLog(LocalMarginValueLoadIncrease, 0.) << Trace::endline;
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//     return;  // unable to launch the initial simulation with 0% of load increase
//   }

//   // analyze results
//   for (std::vector<SimulationResult>::const_iterator it = results_[idx].begin(),
//       itEnd = results_[idx].end(); it != itEnd; ++it) {
//     Trace::info(logTag_) <<  DYNAlgorithmsLog(ScenariosEnd, it->getUniqueScenarioId(), getStatusAsString(it->getStatus())) << Trace::endline;
//     if (it->getStatus() != CONVERGENCE_STATUS) {  // one event crashes
//       Trace::info(logTag_) << "============================================================ " << Trace::endline;
//       Trace::info(logTag_) << DYNAlgorithmsLog(LocalMarginValueScenario, it->getUniqueScenarioId(), 0.) << Trace::endline;
//       Trace::info(logTag_) << "============================================================ " << Trace::endline;
//       return;
//     }
//   }
//   Trace::info(logTag_) << Trace::endline;

//   if (marginCalculation->getCalculationType() == MarginCalculation::GLOBAL_MARGIN || events.size() == 1) {
//     double value = computeGlobalMargin(loadIncrease, baseJobsFile, events, maximumVariationPassing, marginCalculation->getAccuracy());
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//     Trace::info(logTag_) << DYNAlgorithmsLog(GlobalMarginValue, value) << Trace::endline;
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//   } else {
//     assert(marginCalculation->getCalculationType() == MarginCalculation::LOCAL_MARGIN);
//     std::vector<double> results(events.size(), 0);
//     double value = computeLocalMargin(loadIncrease, baseJobsFile, events, marginCalculation->getAccuracy(), results);
//     if (result100.getSuccess()) {
//       value = 100;
//     }
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//     Trace::info(logTag_) << DYNAlgorithmsLog(LocalMarginValueLoadIncrease, value) << Trace::endline;
//     for (size_t i = 0, iEnd = results.size(); i < iEnd; ++i) {
//       Trace::info(logTag_) << DYNAlgorithmsLog(LocalMarginValueScenario, events[i]->getId(), results[i]) << Trace::endline;
//     }
//     Trace::info(logTag_) << "============================================================ " << Trace::endline;
//   }
// }

double
MarginCalculationLauncher::computeGlobalMargin(const boost::shared_ptr<LoadIncrease>& loadIncrease,
    const std::string& baseJobsFile, const std::vector<boost::shared_ptr<Scenario> >& events,
    std::vector<double >& maximumVariationPassing, double tolerance) {
  double minVariation = 0;
  double maxVariation = 100;

  while ( maxVariation - minVariation > tolerance ) {
    double newVariation = round((minVariation + maxVariation)/2);
    results_.push_back(LoadIncreaseResult());
    size_t idx = results_.size() - 1;
    results_[idx].resize(events.size());
    results_[idx].setLoadLevel(newVariation);
    SimulationResult result;
    findOrLaunchLoadIncrease(loadIncrease, newVariation, tolerance, result);
    results_[idx].setStatus(result.getStatus());
    if (result.getSuccess()) {
      std::queue< task_t > toRun;
      std::vector<size_t> eventsIds;
      for (size_t i=0, iEnd = events.size(); i < iEnd ; i++) {
        if (newVariation > maximumVariationPassing[i]) {
          eventsIds.push_back(i);
        } else {
          results_[idx].getResult(i).setScenarioId(events[i]->getId());
          results_[idx].getResult(i).setVariation(newVariation);
          results_[idx].getResult(i).setSuccess(true);
          results_[idx].getResult(i).setStatus(CONVERGENCE_STATUS);
        }
      }
      findAllLevelsBetween(minVariation, maxVariation, tolerance, eventsIds, toRun);
      findOrLaunchScenarios(baseJobsFile, events, toRun, results_[idx]);

      // analyze results
      unsigned int nbSuccess = 0;
      size_t id = 0;
      for (std::vector<SimulationResult>::const_iterator it = results_[idx].begin(),
          itEnd = results_[idx].end(); it != itEnd; ++it, ++id) {
        if (newVariation < maximumVariationPassing[id] || DYN::doubleEquals(newVariation, maximumVariationPassing[id]))
          Trace::info(logTag_) << DYNAlgorithmsLog(ScenarioNotSimulated, it->getUniqueScenarioId()) << Trace::endline;
        else
          Trace::info(logTag_) << DYNAlgorithmsLog(ScenariosEnd, it->getUniqueScenarioId(), getStatusAsString(it->getStatus())) << Trace::endline;
        if (it->getStatus() == CONVERGENCE_STATUS || newVariation < maximumVariationPassing[id] ||
          DYN::doubleEquals(newVariation, maximumVariationPassing[id])) {  // event OK
          nbSuccess++;
          if (newVariation > maximumVariationPassing[id])
            maximumVariationPassing[id] = newVariation;
        }
      }
      if (nbSuccess == events.size() )  // all events succeed
        minVariation = newVariation;
      else
        maxVariation = newVariation;  // at least, one crash
    } else {
      maxVariation = newVariation;  // load increase crashed
    }
    Trace::info(logTag_) << Trace::endline;
  }
  return minVariation;
}

void
MarginCalculationLauncher::findAllLevelsBetween(const double minVariation, const double maxVariation, const double tolerance,
    const std::vector<size_t>& eventIdxs, std::queue< task_t >& toRun) {
  auto& context = mpi::context();
  unsigned nbMaxToAdd = context.nbProcs()/eventIdxs.size();
  if (nbMaxToAdd == 0) nbMaxToAdd = 1;
  double newVariation = round((minVariation + maxVariation)/2);
  std::queue< std::pair<double, double> > minMaxStack;

  toRun.push(task_t(minVariation, maxVariation, eventIdxs));
  if (maxVariation - newVariation > tolerance)
    minMaxStack.push(std::make_pair(newVariation, maxVariation));
  while (toRun.size() < nbMaxToAdd && !minMaxStack.empty()) {
    double min = minMaxStack.front().first;
    double max = minMaxStack.front().second;
    minMaxStack.pop();
    double nextVar = round((min + max)/2);
    auto it = loadIncreaseStatus_.find(nextVar);
    if (it == loadIncreaseStatus_.end() || !it->second.success) continue;
    toRun.push(task_t(min, max, eventIdxs));
    if (max - nextVar > tolerance)
      minMaxStack.push(std::make_pair(nextVar, max));
    if (nextVar - min > tolerance)
      minMaxStack.push(std::make_pair(min, nextVar));
  }
}

double
MarginCalculationLauncher::computeLocalMargin(const boost::shared_ptr<LoadIncrease>& loadIncrease,
    const std::string& baseJobsFile, const std::vector<boost::shared_ptr<Scenario> >& events, double tolerance,
    std::vector<double>& results) {
  double maxLoadVarForLoadIncrease = 0.;
  std::queue< task_t > toRun;
  std::vector<size_t> all;
  for (size_t i=0, iEnd = events.size(); i < iEnd ; i++)
    all.push_back(i);
  toRun.push(task_t(0, 100, all));

  while (!toRun.empty()) {
    std::queue< task_t > toRunCopy(toRun);  // Needed as findOrLaunchScenarios modifies the queue
    task_t task = toRun.front();
    toRun.pop();
    const std::vector<size_t>& eventsId = task.ids_;
    double newVariation = round((task.minVariation_ + task.maxVariation_)/2);
    results_.push_back(LoadIncreaseResult());
    size_t idx = results_.size() - 1;
    results_[idx].resize(eventsId.size());
    results_[idx].setLoadLevel(newVariation);
    SimulationResult result;
    findOrLaunchLoadIncrease(loadIncrease, newVariation, tolerance, result);
    results_[idx].setStatus(result.getStatus());

    if (result.getSuccess()) {
      if (maxLoadVarForLoadIncrease < newVariation)
        maxLoadVarForLoadIncrease = newVariation;
      LoadIncreaseResult liResultTmp;
      liResultTmp.resize(events.size());
      findOrLaunchScenarios(baseJobsFile, events, toRunCopy, liResultTmp);

      // analyze results
      task_t below(task.minVariation_, newVariation);
      task_t above(newVariation, task.maxVariation_);
      for (size_t i = 0, iEnd = eventsId.size(); i < iEnd; ++i) {
        results_[idx].getResult(i) = liResultTmp.getResult(eventsId[i]);
        Trace::info(logTag_) << DYNAlgorithmsLog(ScenariosEnd,
            results_[idx].getResult(i).getUniqueScenarioId(), getStatusAsString(results_[idx].getResult(i).getStatus())) << Trace::endline;
        if (results_[idx].getResult(i).getStatus() == CONVERGENCE_STATUS) {  // event OK
          if (results[eventsId[i]] < newVariation)
            results[eventsId[i]] = newVariation;
          if ( task.maxVariation_ - newVariation > tolerance ) {
            above.ids_.push_back(eventsId[i]);
          }
        } else {
          if ( newVariation - task.minVariation_ > tolerance )
            below.ids_.push_back(eventsId[i]);
        }
      }
      if (!below.ids_.empty())
        toRun.push(below);
      if (!above.ids_.empty())
        toRun.push(above);
    } else if ( newVariation - task.minVariation_ > tolerance ) {
      task_t below(task.minVariation_, newVariation);
      for (size_t i = 0, iEnd = eventsId.size(); i < iEnd; ++i) {
        below.ids_.push_back(eventsId[i]);
      }
      if (!below.ids_.empty())
        toRun.push(below);
    }
    Trace::info(logTag_) << Trace::endline;
  }
  return maxLoadVarForLoadIncrease;
}

void MarginCalculationLauncher::findOrLaunchScenarios(const std::string& baseJobsFile,
    const std::vector<boost::shared_ptr<Scenario> >& events,
    std::queue< task_t >& toRun,
    LoadIncreaseResult& result) {
  if (toRun.empty()) return;
  task_t task = toRun.front();
  toRun.pop();
  const std::vector<size_t>& eventsId = task.ids_;
  double newVariation = round((task.minVariation_ + task.maxVariation_)/2);
  if (mpi::context().nbProcs() == 1) {
    std::string iidmFile = generateIDMFileNameForVariation(newVariation);
    if (inputsByIIDM_.count(iidmFile) == 0) {
      // read inputs only if not already existing with enough variants defined
      inputsByIIDM_[iidmFile].readInputs(workingDirectory_, baseJobsFile, iidmFile);
    }
    for (unsigned int i=0; i < eventsId.size(); i++) {
      launchScenario(inputsByIIDM_[iidmFile], events[eventsId[i]], newVariation, result.getResult(eventsId[i]));
    }
    return;
  }

  auto found = scenarioStatus_.find(newVariation);
  if (found != scenarioStatus_.end()) {
    Trace::info(logTag_) << DYNAlgorithmsLog(ScenarioResultsFound, newVariation) << Trace::endline;
    for (const auto& eventId : eventsId) {
      auto resultId = SimulationResult::getUniqueScenarioId(events.at(eventId)->getId(), newVariation);
      result.getResult(eventId) = importResult(resultId);
    }
    return;
  }

  std::vector<std::pair<size_t, double> > events2Run;
  prepareEvents2Run(task, toRun, events2Run);

  for (std::vector<std::pair<size_t, double> >::const_iterator it = events2Run.begin(); it != events2Run.end(); ++it) {
    double variation = it->second;
    std::string iidmFile = generateIDMFileNameForVariation(variation);
    if (inputsByIIDM_.count(iidmFile) == 0) {
      inputsByIIDM_[iidmFile].readInputs(workingDirectory_, baseJobsFile, iidmFile);
    }
  }

  std::vector<bool> successes;
  mpi::forEach(0, events2Run.size(), [this, &events2Run, &events, &successes](unsigned int i){
    double variation = events2Run[i].second;
    std::string iidmFile = generateIDMFileNameForVariation(variation);
    size_t eventIdx = events2Run[i].first;
    SimulationResult result;
    createScenarioWorkingDir(events.at(eventIdx)->getId(), variation);
    launchScenario(inputsByIIDM_.at(iidmFile), events.at(eventIdx), variation, result);
    successes.push_back(result.getSuccess());
    exportResult(result);
  });
  // Sync successes
  std::vector<bool> allSuccesses = synchronizeSuccesses(successes);
  for (unsigned int i = 0; i < events2Run.size(); i++) {
    auto& event = events2Run.at(i);
    // variation = event.second
    scenarioStatus_[event.second][event.first].success = allSuccesses.at(i);
  }
  assert(scenarioStatus_.count(newVariation) > 0);

  for (const auto& eventId : eventsId) {
    auto resultId = SimulationResult::getUniqueScenarioId(events.at(eventId)->getId(), newVariation);
    result.getResult(eventId) = importResult(resultId);
  }

  for (unsigned int i=0; i < events2Run.size(); i++) {
    double variation = events2Run[i].second;
    std::string iidmFile = generateIDMFileNameForVariation(variation);
    inputsByIIDM_.erase(iidmFile);  // remove iidm file used for scenario to save RAM
  }
}

void
MarginCalculationLauncher::prepareEvents2Run(const task_t& requestedTask,
    std::queue< task_t >& toRun,
    std::vector<std::pair<size_t, double> >& events2Run) {
  const std::vector<size_t>& eventsId = requestedTask.ids_;
  double newVariation = round((requestedTask.minVariation_ + requestedTask.maxVariation_)/2);
  for (size_t i = 0, iEnd = eventsId.size(); i < iEnd; ++i) {
    events2Run.push_back(std::make_pair(eventsId[i], newVariation));
  }
  while (events2Run.size() < mpi::context().nbProcs() && !toRun.empty()) {
    task_t newTask = toRun.front();
    toRun.pop();
    const std::vector<size_t>& newEventsId = newTask.ids_;
    double variation = round((newTask.minVariation_ + newTask.maxVariation_)/2);
    auto it = loadIncreaseStatus_.find(variation);
    if (it != loadIncreaseStatus_.end() && !it->second.success) continue;
    if (scenarioStatus_.find(variation) != scenarioStatus_.end()) continue;
    for (size_t i = 0, iEnd = newEventsId.size(); i < iEnd; ++i) {
      events2Run.push_back(std::make_pair(newEventsId[i], variation));
    }
  }
}

void
MarginCalculationLauncher::launchScenario(const MultiVariantInputs& inputs, const boost::shared_ptr<Scenario>& scenario,
    const double variation, SimulationResult& result) {
  if (mpi::context().nbProcs() == 1)
    std::cout << " Launch task :" << scenario->getId() << " dydFile =" << scenario->getDydFile()
              << " criteriaFile =" << scenario->getCriteriaFile() << std::endl;

  std::stringstream subDir;
  subDir << "step-" << variation << "/" << scenario->getId();
  std::string workingDir = createAbsolutePath(subDir.str(), workingDirectory_);
  boost::shared_ptr<job::JobEntry> job = inputs.cloneJobEntry();
  addDydFileToJob(job, scenario->getDydFile());
  setCriteriaFileForJob(job, scenario->getCriteriaFile());

  SimulationParameters params;
  std::stringstream dumpFile;
  dumpFile << workingDirectory_ << "/loadIncreaseFinalState-" << variation << ".dmp";
  //  force simulation to load previous dump and to use final values
  params.InitialStateFile_ = dumpFile.str();
  params.iidmFile_ = generateIDMFileNameForVariation(variation);

  // startTime and stopTime are adapted depending on the variation length
  double startTime = tLoadIncrease_ - (100. - variation)/100. * inputs_.getTLoadIncreaseVariationMax();
  params.startTime_ = startTime;
  params.stopTime_ = startTime + tScenario_;

  result.setScenarioId(scenario->getId());
  result.setVariation(variation);
  boost::shared_ptr<DYN::Simulation> simulation = createAndInitSimulation(workingDir, job, params, result, inputs);

  if (simulation) {
    simulation->setTimelineOutputFile("");
    simulation->setConstraintsOutputFile("");
    // The event time should be adapted (the list of events models supported currently corresponds to events really used)
    boost::shared_ptr<DYN::ModelMulti> modelMulti = boost::dynamic_pointer_cast<DYN::ModelMulti>(simulation->model_);
    std::string DDBDir = getEnvVar("DYNAWO_DDB_DIR");
    std::vector<boost::shared_ptr<DYN::SubModel> > subModels = modelMulti->findSubModelByLib(DDBDir + "/EventQuadripoleDisconnection"
      + DYN::sharedLibraryExtension());
    std::vector<boost::shared_ptr<DYN::SubModel> > subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventConnectedStatus"
      + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventSetPointBoolean" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/SetPoint" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventSetPointReal" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventSetPointDoubleReal" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventSetPointGenerator" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventSetPointLoad" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/LineTrippingEvent" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/TfoTrippingEvent" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventConnectedStatus" + DYN::sharedLibraryExtension());
    subModels.insert(subModels.end(), subModelsToAdd.begin(), subModelsToAdd.end());
    subModelsToAdd = modelMulti->findSubModelByLib(DDBDir + "/EventQuadripoleConnection" + DYN::sharedLibraryExtension());
    for (std::vector<boost::shared_ptr<DYN::SubModel> >::const_iterator it = subModels.begin(); it != subModels.end(); ++it) {
      double tEvent = (*it)->findParameterDynamic("event_tEvent").getValue<double>();
      (*it)->setParameterValue("event_tEvent", DYN::PAR, tEvent - (100. - variation) * inputs_.getTLoadIncreaseVariationMax() / 100., false);
      (*it)->setSubModelParameters();
    }
    simulate(simulation, result);
  }

  if (mpi::context().nbProcs() == 1)
    std::cout << " Task :" << scenario->getId() << " status =" << getStatusAsString(result.getStatus()) << std::endl;
}

std::vector<double>
MarginCalculationLauncher::generateVariationsToLaunch(unsigned int maxNumber, double variation, double tolerance) const {
  std::vector<double> variationsToLaunch;
  if (loadIncreaseStatus_.empty()) {
    // First time we call this, we know we have at least 2 threads.
    variationsToLaunch.push_back(0.);
    variationsToLaunch.push_back(100.);
  }
  if (variationsToLaunch.size() < maxNumber) {
    std::queue< std::pair<double, double> > levels;
    double closestVariationBelow = 0.;
    double closestVariationAbove = 100.;
    for (const auto& status : loadIncreaseStatus_) {
      if (!status.second.success) {
        continue;
      }
      if (closestVariationBelow < status.first && status.first < variation)
        closestVariationBelow = status.first;
      if (status.first < closestVariationAbove &&  variation < status.first)
        closestVariationAbove = status.first;
    }
    levels.push(std::make_pair(closestVariationBelow, closestVariationAbove));
    while (!levels.empty() && variationsToLaunch.size() < maxNumber) {
      std::pair<double, double> currentLevel = levels.front();
      levels.pop();
      double nextVariation = round((currentLevel.first + currentLevel.second)/2);
      variationsToLaunch.push_back(nextVariation);
      if (currentLevel.second - nextVariation > tolerance)
        levels.push(std::make_pair(nextVariation, currentLevel.second));
      if (DYN::doubleNotEquals(nextVariation, 50.) && nextVariation - currentLevel.first > tolerance)
        levels.push(std::make_pair(currentLevel.first, nextVariation));
    }
  }

  return variationsToLaunch;
}

std::string
MarginCalculationLauncher::computeLoadIncreaseScenarioId(double variation) {
  std::stringstream ss;
  ss << "loadIncrease-" << variation;
  return ss.str();
}

std::vector<bool>
MarginCalculationLauncher::synchronizeSuccesses(const std::vector<bool>& successes) {
  auto& context = mpi::context();
  std::vector<std::vector<bool>> gatheredSuccesses;
  context.gather(successes, gatheredSuccesses);
  std::vector<bool> allSuccesses;
  if (context.isRootProc()) {
    auto size =
      std::accumulate(gatheredSuccesses.begin(), gatheredSuccesses.end(), 0, [](size_t sum, const std::vector<bool>& data) { return sum + data.size(); });
    allSuccesses.resize(size);
    for (unsigned int i = 0; i < context.nbProcs(); i++) {
      const auto& vect = gatheredSuccesses.at(i);
      for (unsigned int j = 0; j < vect.size(); j++) {
        // variations were attributed to procs following the formula: "index % nbprocs == rank" throught forEach function
        allSuccesses.at(j * context.nbProcs() + i) = vect.at(j);
      }
    }
  }
  context.broadcast(allSuccesses);
  return allSuccesses;
}

void
MarginCalculationLauncher::findOrLaunchLoadIncrease(const boost::shared_ptr<LoadIncrease>& loadIncrease,
    const double variation, const double tolerance, SimulationResult& result) {
  Trace::info(logTag_) << DYNAlgorithmsLog(VariationValue, variation) << Trace::endline;
  if (mpi::context().nbProcs() == 1) {
    inputs_.readInputs(workingDirectory_, loadIncrease->getJobsFile());
    launchLoadIncrease(loadIncrease, variation, result);
    return;
  }

  auto found = loadIncreaseStatus_.find(variation);
  if (found != loadIncreaseStatus_.end()) {
    result = importResult(computeLoadIncreaseScenarioId(variation));
    Trace::info(logTag_) << DYNAlgorithmsLog(LoadIncreaseResultsFound, variation) << Trace::endline;
    return;
  }

  // Algo to generate variations to launch
  auto& context = mpi::context();
  std::vector<double> variationsToLaunch = generateVariationsToLaunch(context.nbProcs(), variation, tolerance);

  // Launch Simulations
  inputs_.readInputs(workingDirectory_, loadIncrease->getJobsFile());
  std::vector<bool> successes;
  mpi::forEach(0, variationsToLaunch.size(), [this, &loadIncrease, &variationsToLaunch, &successes](unsigned int i){
    SimulationResult result;
    launchLoadIncrease(loadIncrease, variationsToLaunch.at(i), result);
    successes.push_back(result.getSuccess());
    exportResult(result);
  });
  // Sync successes
  std::vector<bool> allSuccesses = synchronizeSuccesses(successes);
  // Fill load increase status
  for (unsigned int i = 0; i < variationsToLaunch.size(); i++) {
    loadIncreaseStatus_.insert(std::make_pair(variationsToLaunch.at(i), LoadIncreaseStatus(allSuccesses.at(i))));
  }
  assert(loadIncreaseStatus_.count(variation) > 0);
  result = importResult(computeLoadIncreaseScenarioId(variation));
}

void
MarginCalculationLauncher::launchLoadIncrease(const boost::shared_ptr<LoadIncrease>& loadIncrease,
    const double variation, SimulationResult& result) {
  if (mpi::context().nbProcs() == 1)
    std::cout << "Launch loadIncrease of " << variation << "%" <<std::endl;

  std::stringstream subDir;
  subDir << "step-" << variation << "/" << loadIncrease->getId();
  std::string workingDir = createAbsolutePath(subDir.str(), workingDirectory_);
  boost::shared_ptr<job::JobEntry> job = inputs_.cloneJobEntry();
  job->getOutputsEntry()->setTimelineEntry(boost::shared_ptr<job::TimelineEntry>());
  job->getOutputsEntry()->setConstraintsEntry(boost::shared_ptr<job::ConstraintsEntry>());

  SimulationParameters params;
  //  force simulation to dump final values (would be used as input to launch each event)
  params.activateDumpFinalState_ = true;
  params.activateExportIIDM_ = true;
  std::stringstream iidmFile;
  iidmFile << workingDirectory_ << "/loadIncreaseFinalState-" << variation << ".iidm";
  params.exportIIDMFile_ = iidmFile.str();
  std::stringstream dumpFile;
  dumpFile << workingDirectory_ << "/loadIncreaseFinalState-" << variation << ".dmp";
  params.dumpFinalStateFile_ = dumpFile.str();

  result.setScenarioId(computeLoadIncreaseScenarioId(variation));
  boost::shared_ptr<DYN::Simulation> simulation = createAndInitSimulation(workingDir, job, params, result, inputs_);

  if (simulation) {
    boost::shared_ptr<DYN::ModelMulti> modelMulti = boost::dynamic_pointer_cast<DYN::ModelMulti>(simulation->model_);
    std::string DDBDir = getMandatoryEnvVar("DYNAWO_DDB_DIR");
    std::vector<boost::shared_ptr<DYN::SubModel> > subModels = modelMulti->findSubModelByLib(DDBDir + "/DYNModelVariationArea" + DYN::sharedLibraryExtension());
    for (std::vector<boost::shared_ptr<DYN::SubModel> >::const_iterator it = subModels.begin(); it != subModels.end(); ++it) {
      double startTime = (*it)->findParameterDynamic("startTime").getValue<double>();
      double stopTime = (*it)->findParameterDynamic("stopTime").getValue<double>();
      inputs_.setTLoadIncreaseVariationMax(stopTime - startTime);
      int nbLoads = (*it)->findParameterDynamic("nbLoads").getValue<int>();
      for (int k = 0; k < nbLoads; ++k) {
        std::stringstream deltaPName;
        deltaPName << "deltaP_load_" << k;
        double deltaP = (*it)->findParameterDynamic(deltaPName.str()).getValue<double>();
        (*it)->setParameterValue(deltaPName.str(), DYN::PAR, deltaP*variation/100., false);

        std::stringstream deltaQName;
        deltaQName << "deltaQ_load_" << k;
        double deltaQ = (*it)->findParameterDynamic(deltaQName.str()).getValue<double>();
        (*it)->setParameterValue(deltaQName.str(), DYN::PAR, deltaQ*variation/100., false);
      }
      // change of the stop time to keep the same ramp of variation.
      double originalDuration = stopTime - startTime;
      double newStopTime = startTime + originalDuration * variation / 100.;
      (*it)->setParameterValue("stopTime", DYN::PAR, newStopTime, false);
      (*it)->setSubModelParameters();  // update values stored in subModel
      Trace::info(logTag_) << DYNAlgorithmsLog(LoadIncreaseModelParameter, (*it)->name(), newStopTime, variation/100.) << Trace::endline;
    }
    simulation->setStopTime(tLoadIncrease_ - (100. - variation)/100. * inputs_.getTLoadIncreaseVariationMax());
    simulate(simulation, result);
  }
}

void
MarginCalculationLauncher::createOutputs(std::map<std::string, std::string>& mapData, bool zipIt) const {
  Trace::resetCustomAppenders();  // to force flush
  Trace::resetPersistantCustomAppenders();  // to force flush
  aggregatedResults::XmlExporter exporter;
  if (zipIt) {
    std::stringstream aggregatedResults;
    exporter.exportLoadIncreaseResultsToStream(results_, aggregatedResults);
    mapData["aggregatedResults.xml"] = aggregatedResults.str();
    std::ifstream inFile(createAbsolutePath("dynawo.log", workingDirectory_).c_str());
    if (inFile.is_open()) {
      std::string line;
      std::stringstream strStream;
      while (getline(inFile, line)) {
        strStream << line << "\n";
      }
      mapData["dynawo.log"] = strStream.str();
      inFile.close();
    }
  } else {
    exporter.exportLoadIncreaseResultsToFile(results_, outputFileFullPath_);
  }

  std::map<std::string, SimulationResult> bestResults;
  std::map<std::string, SimulationResult> worstResults;
  for (std::vector<LoadIncreaseResult>::const_iterator itLoadIncreaseResult = results_.begin();
       itLoadIncreaseResult != results_.end(); ++itLoadIncreaseResult) {
    double loadLevel = itLoadIncreaseResult->getLoadLevel();
    for (std::vector<SimulationResult>::const_iterator itSimulationResult = itLoadIncreaseResult->begin();
         itSimulationResult != itLoadIncreaseResult->end(); ++itSimulationResult) {
      std::string scenarioId = itSimulationResult->getScenarioId();
      if (itSimulationResult->getSuccess()) {
        std::map<std::string, SimulationResult>::iterator itBest = bestResults.find(scenarioId);
        if (itBest == bestResults.end() || (loadLevel > itBest->second.getVariation()))
          bestResults[scenarioId] = *itSimulationResult;
      } else {
        std::map<std::string, SimulationResult>::iterator itWorst = worstResults.find(scenarioId);
        if (itWorst == worstResults.end() || loadLevel < itWorst->second.getVariation())
          worstResults[scenarioId] = *itSimulationResult;
      }
    }
  }

  for (std::map<std::string, SimulationResult>::iterator itBest = bestResults.begin();
       itBest != bestResults.end(); ++itBest) {
    if (zipIt) {
      storeOutputs(itBest->second, mapData);
    } else {
      writeOutputs(itBest->second);
    }
  }
  for (std::map<std::string, SimulationResult>::iterator itWorst = worstResults.begin();
       itWorst != worstResults.end(); ++itWorst) {
    if (zipIt) {
      storeOutputs(itWorst->second, mapData);
    } else {
      writeOutputs(itWorst->second);
    }
  }
}

std::string
MarginCalculationLauncher::generateIDMFileNameForVariation(double variation) const {
  std::stringstream iidmFile;
  iidmFile << workingDirectory_ << "/loadIncreaseFinalState-" << variation << ".iidm";
  return iidmFile.str();
}


}  // namespace DYNAlgorithms

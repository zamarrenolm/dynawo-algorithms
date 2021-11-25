#include "DYNMarginSimulation.h"

#include "DYNMarginCalculation.h"
#include "DYNScenario.h"
#include "DYNScenarios.h"

namespace DYNAlgorithms {

MarginSimulation::MarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs) : jobs_(jobs) {}

GlobalMarginSimulation::GlobalMarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs) :
    MarginSimulation(jobs),
    generator_(variationStart_, variationEnd_, jobs->getMarginCalculation()->getAccuracy()) {}

LocalMarginSimulation::LocalMarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs) : MarginSimulation(jobs) {
  const auto& scenarios = jobs->getMarginCalculation()->getScenarios()->getScenarios();
  for (const auto& scenario : scenarios) {
    generators_.emplace(std::make_pair(scenario->getId(), VariationGenerator(variationStart_, variationEnd_, jobs->getMarginCalculation()->getAccuracy())));
  }
}

}  // namespace DYNAlgorithms

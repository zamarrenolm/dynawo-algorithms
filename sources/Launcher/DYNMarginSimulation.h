#ifndef LAUNCHER_DYNMARGINSIMULATION_H_
#define LAUNCHER_DYNMARGINSIMULATION_H_

#include "DYNMultipleJobs.h"
#include "DYNVariationGenerator.h"

#include <boost/shared_ptr.hpp>
#include <string>
#include <unordered_map>

namespace DYNAlgorithms {

class MarginSimulation {
 public:
  using ScenarioId = std::string;

 public:
  explicit MarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs);
  virtual ~MarginSimulation() = default;

  const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs() const {
    return jobs_;
  }

  virtual std::vector<unsigned int> generateVariationAny(unsigned int maxNumber) const = 0;
  virtual void validVariationAll(unsigned int variation, bool valid) = 0;

 protected:
  static constexpr unsigned int variationStart_ = 0;
  static constexpr unsigned int variationEnd_ = 100;

 protected:
  boost::shared_ptr<multipleJobs::MultipleJobs> jobs_;
};

class GlobalMarginSimulation : public MarginSimulation {
 public:
  explicit GlobalMarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs);

  std::vector<unsigned int> generateVariationAny(unsigned int maxNumber) const final {
    return generator_.generate(maxNumber);
  }

  void validVariationAll(unsigned int variation, bool valid) final {
    generator_.valid(variation, valid);
  }

  VariationGenerator& variationGenerator() {
    return generator_;
  }
  const VariationGenerator& variationGenerator() const {
    return generator_;
  }

 private:
  VariationGenerator generator_;
};

class LocalMarginSimulation : public MarginSimulation {
 public:
  explicit LocalMarginSimulation(const boost::shared_ptr<multipleJobs::MultipleJobs>& jobs);

  std::vector<unsigned int> generateVariationAny(unsigned int maxNumber) const final {
    return generators_.begin()->second.generate(maxNumber);
  }

  void validVariationAll(unsigned int variation, bool valid) final {
    for (auto& pair : generators_) {
      pair.second.valid(variation, valid);
    }
  }

  VariationGenerator& variationGenerator(const ScenarioId& id) {
    return generators_.at(id);
  }
  const VariationGenerator& variationGenerator(const ScenarioId& id) const {
    return generators_.at(id);
  }

 private:
  std::unordered_map<ScenarioId, VariationGenerator> generators_;
};

}  // namespace DYNAlgorithms

#endif  // LAUNCHER_DYNMARGINSIMULATION_H_

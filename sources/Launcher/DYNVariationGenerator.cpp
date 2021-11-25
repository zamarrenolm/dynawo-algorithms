#include "DYNVariationGenerator.h"

#include "DYNCommon.h"

#include <cassert>

namespace DYNAlgorithms {
VariationGenerator::VariationGenerator(unsigned int min, unsigned int max, unsigned int accuracy) :
    min_(min),
    max_(max),
    accuracy_(accuracy),
    range_{min, max} {}

bool
VariationGenerator::isInRange(unsigned int value, unsigned int min, unsigned int max) {
  return value >= min && value <= max;
}

void
VariationGenerator::valid(unsigned int variation, bool isValid) {
  // variation must be in (min, max)
  if (!isInRange(variation, range_.front(), range_.back())) {
    return;
  }

  unsigned int index = isValid ? 0 : 1;  // valid => we update the minimum
  range_.at(index) = variation;
}

std::vector<unsigned int>
VariationGenerator::generate(unsigned int max) const {
  std::vector<unsigned int> ret;

  if (max == 0) {
    return ret;
  }

  unsigned int step = std::floor((range_.back() - range_.front()) / (max + 1));
  if (step < accuracy_)
    step = accuracy_;

  unsigned int nbItems = 0;
  unsigned int variation = computeNextVariation(range_.back(), step);  // max is considered already done
  while (isInRange(variation, range_.front(), range_.back()) && nbItems < max) {
    ret.push_back(variation);
    variation = computeNextVariation(variation, step);
    nbItems++;
  }

  return ret;
}

}  // namespace DYNAlgorithms

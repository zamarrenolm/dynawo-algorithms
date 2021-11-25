#ifndef LAUNCHER_DYNVARIATIONGENERATOR_H_
#define LAUNCHER_DYNVARIATIONGENERATOR_H_

#include <array>
#include <vector>

namespace DYNAlgorithms {
class VariationGenerator {
 public:
  VariationGenerator(unsigned int min, unsigned int max, unsigned int accuracy);

  std::vector<unsigned int> generate(unsigned int max) const;
  void valid(unsigned int variation, bool isValid);
  unsigned int distance() const {
    return range_.back() - range_.front();
  }
  unsigned int min() const {
    return range_.front();
  }

 private:
  static bool isInRange(unsigned int value, unsigned int min, unsigned int max);
  unsigned int computeNextVariation(unsigned int var, unsigned int step) const {
    return (var - step) / accuracy_ * accuracy_;
  }

 private:
  const unsigned int min_;
  const unsigned int max_;
  const unsigned int accuracy_;
  std::array<unsigned int, 2> range_;
};
}  // namespace DYNAlgorithms

#endif  // LAUNCHER_DYNVARIATIONGENERATOR_H_

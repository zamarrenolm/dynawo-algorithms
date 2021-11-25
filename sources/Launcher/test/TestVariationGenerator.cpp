#include "DYNVariationGenerator.h"
#include "gtest_dynawo.h"

namespace DYNAlgorithms {

TEST(Variations, base) {
  VariationGenerator generator(0, 100, 3);

  std::vector<unsigned int> expectedVariations = {48};
  auto vec = generator.generate(1);
  ASSERT_EQ(vec, expectedVariations);

  generator.valid(48, true);  // range is [48, 100]
  expectedVariations = {72};
  vec = generator.generate(1);
  ASSERT_EQ(vec, expectedVariations);
}

TEST(Variations, min) {
  VariationGenerator generator(97, 100, 3);

  auto vec = generator.generate(1);
  ASSERT_TRUE(vec.empty());
  ASSERT_EQ(generator.min(), 97);
}

TEST(Variations, maxNumber) {
  VariationGenerator generator(0, 100, 3);

  std::vector<unsigned int> expectedVariations = {75, 48, 21};
  auto vec = generator.generate(3);
  ASSERT_EQ(vec, expectedVariations);
  vec = generator.generate(3);
  ASSERT_EQ(vec, expectedVariations);

  generator.valid(21, true);    // range is [21, 100]
  generator.valid(48, true);    // range is [48, 100]
  generator.valid(75, false);   // range is [48, 75]
  generator.valid(100, false);  // range is [48, 75]
  ASSERT_EQ(generator.distance(), 27);
  expectedVariations = {69, 63, 57, 51};
  vec = generator.generate(4);
  ASSERT_EQ(vec, expectedVariations);

  generator.valid(69, true);  // range is [69, 75]
  expectedVariations = {72, 69};
  vec = generator.generate(4);
  ASSERT_EQ(vec, expectedVariations);
  generator.valid(72, true);  // range is [72, 75]
  ASSERT_EQ(generator.distance(), 3);
}

}  // namespace DYNAlgorithms

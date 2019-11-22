#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_non_pure_lambdas) {
  {
    std::string testing_string;
    std::function<int(int, int)> non_pure_lambda = [&testing_string](int a, int b) -> int {
      testing_string = "Tested!";
      return a + b;
    };

    functor_t f_non_pure_lambda(non_pure_lambda);

    int result = f_non_pure_lambda(5u, 5u);

    EXPECT_EQ(result, 5u + 5u);
    EXPECT_STREQ(testing_string.c_str(), "Tested!");
  }
}

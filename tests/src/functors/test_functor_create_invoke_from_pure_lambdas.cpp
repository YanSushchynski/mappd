#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_pure_lambdas) {
  {
    std::function<int(int, int)> pure_lambda = [](int a, int b) -> int {
      return a + b;
    };
    functor_t f_pure_lambda(pure_lambda);

    int result = f_pure_lambda(5u, 5u);

    EXPECT_EQ(result, pure_lambda(5u, 5u));
  }
}

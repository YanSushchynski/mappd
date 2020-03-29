#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_pure_lambdas) {
  {
    std::function<int32_t(int32_t, int32_t)> pure_lambda = [](int32_t a, int32_t b) -> int32_t {
      return a + b;
    };
	
    functor_t f_pure_lambda(pure_lambda);
    int32_t result = f_pure_lambda(5u, 5u);

    EXPECT_EQ(result, pure_lambda(5u, 5u));
  }
}

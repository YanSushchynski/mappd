#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_static_member_function) {
  {
    class Test {
    public:
      int32_t sum(int32_t a, int32_t b) { return a + b; }

      static int32_t sum_static(int32_t a, int32_t b) { return a + b; }
    };

    functor_t f_member_static(Test::sum_static);
    int32_t result = f_member_static(5u, 5u);

    EXPECT_EQ(result, 5u + 5u);
  }
}

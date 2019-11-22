#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_static_member_function) {
  {
    class Test {
    public:
      int sum(int a, int b) { return a + b; }

      static int sum_static(int a, int b) { return a + b; }
    };

    functor_t f_member_static(Test::sum_static);

    int result = f_member_static(5u, 5u);

    EXPECT_EQ(result, 5u + 5u);
  }
}

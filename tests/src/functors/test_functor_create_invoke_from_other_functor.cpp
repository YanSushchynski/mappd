#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_others) {
  {
    std::string testing_string;

    std::function<void(int, int)> non_pure_void_lambda = [&testing_string](int a, int b) -> void {
      testing_string = "Tested!" + std::to_string(a + b);
    };

    std::function<int(int, int)> non_pure_lambda = [&testing_string](int a, int b) -> int {
      testing_string = "Tested!" + std::to_string(a + b);
      return a + b;
    };

    hook_t<void(int, int)> hook_;

    hook_.add("Hook1", non_pure_void_lambda);

    sequence_t sequence_(non_pure_lambda);
    functor_t functor_non_pure_lambda(non_pure_lambda);

    size_t functor_non_pure_lambda_call_count = functor_non_pure_lambda.call_count();
    size_t hook_call_count = hook_.call_count();

    {
      functor_t functor_hook(hook_);
      size_t functor_hook_call_count = functor_hook.call_count();

      functor_hook(5u, 5u);

      EXPECT_STREQ(testing_string.c_str(), "Tested!10");
      EXPECT_EQ(functor_hook_call_count + 1, functor_hook.call_count());

      testing_string.clear();
    }

    {
      functor_t functor_sequence(sequence_);

      size_t functor_sequence_call_count = functor_sequence.call_count();
      int result = functor_sequence(5u, 5u);

      EXPECT_EQ(functor_sequence_call_count + 1u, functor_sequence.call_count());
      EXPECT_EQ(result, 5u + 5u);
    }

    {
      functor_t functor_functor(functor_non_pure_lambda);

      size_t functor_functor_call_count = functor_functor.call_count();
      int result = functor_functor(5u, 5u);

      EXPECT_EQ(result, 5u + 5u);
      EXPECT_STREQ(testing_string.c_str(), "Tested!10");
      EXPECT_EQ(functor_functor_call_count + 1u, functor_functor.call_count());

      testing_string.clear();
    }
  }

  {
    std::function<int(int, int)> pure_lambda = [](int a, int b) -> int { return a + b; };
    std::function<void(int, int)> pure_void_lambda = [](int a, int b) -> void {};

    hook_t<void(int, int)> hook_;

    hook_.add("Hook1", pure_void_lambda);

    sequence_t sequence_(pure_lambda);
    functor_t f_pure_lambda(pure_lambda);

    {
      functor_t f_hook(hook_);
      f_hook(5u, 5u);
    }

    {
      functor_t f_sequence(sequence_);
      int result = f_sequence(5u, 5u);
      EXPECT_EQ(result, 5u + 5u);
    }

    {
      functor_t f_functor(f_pure_lambda);
      int result = f_functor(5u, 5u);
      EXPECT_EQ(result, 5u + 5u);
    }
  }

  {
    class Test {
    public:
      static int static_function(int a, int b) { return a + b; }

      static void static_void_function(int a, int b){};
    };

    hook_t<void(int, int)> hook_;

    hook_.add("Hook1", Test::static_void_function);

    sequence_t sequence_(Test::static_function);
    functor_t f_static_function(Test::static_function);

    {
      functor_t f_hook(hook_);
      f_hook(5u, 5u);
    }

    {
      functor_t f_sequence(sequence_);

      int result = f_sequence(5u, 5u);

      EXPECT_EQ(result, 5u + 5u);
    }

    {
      functor_t f_functor(f_static_function);

      int result = f_functor(5u, 5u);

      EXPECT_EQ(result, 5u + 5u);
    }
  }

  {
    class Test {
    public:
      int member_function(int a, int b) { return a + b; }

      void member_void_function(int a, int b){};
    };

    Test test;
    hook_t<void(int, int)> hook_;

    hook_.add("Hook1", &test, &Test::member_void_function);

    sequence_t sequence_(&test, &Test::member_function);
    functor_t f_member_function(&test, &Test::member_function);

    {
      functor_t f_hook(hook_);
      f_hook(5u, 5u);
    }

    {
      functor_t f_sequence(sequence_);
      int result = f_sequence(5u, 5u);

      EXPECT_EQ(result, 5u + 5u);
    }

    {
      functor_t f_functor(f_member_function);

      int result = f_functor(5u, 5u);

      EXPECT_EQ(result, 5u + 5u);
    }
  }
}

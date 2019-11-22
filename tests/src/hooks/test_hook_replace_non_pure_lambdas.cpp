#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, replace_non_pure_lambdas) {
  {
    hook_t<void(int, int)> hook_;
    std::string captured_string;
    sha256::sha256_hash_type connected_id;

    auto func1 = [&captured_string](int a, int b) -> void {
      captured_string = "FirstSumma = " + std::to_string(a + b);
    };

    auto func2 = [&captured_string](int a, int b) -> void {
      captured_string = "SecondSumma = " + std::to_string(a + b);
    };

    uint32_t hook_error_id = hook_errno_t::HOOK_CLEAR;

    EXPECT_EQ(hook_
                  .set_error_handler([&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                                      const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME)
                                         -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id()),
              hook_errno_t::HOOK_CLEAR);

    {
      auto add_result = hook_.add("SummaFirst", func1);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SummaFirst"), std::strlen("SummaFirst")));

      hook_(5, 5);
      EXPECT_STREQ(captured_string.c_str(), "FirstSumma = 10");

      captured_string.clear();
    }

    {
      auto replace_result = hook_.replace("SummaFirst", "SummaSecond", func2);

      EXPECT_EQ(replace_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SummaSecond"), std::strlen("SummaSecond")));

      hook_(15, 15);
      EXPECT_STREQ(captured_string.c_str(), "SecondSumma = 30");

      captured_string.clear();
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
  }
}

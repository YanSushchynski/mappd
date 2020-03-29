#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, replace_member_functions) {
  {
    class Test {
    public:
      void slot1(int a, int b, std::string &captured_string) {
        captured_string += "FirstSumma = " + std::to_string(a + b);
      }

      void slot2(int a, int b, std::string &captured_string) {
        captured_string = "SecondSumma = " + std::to_string(a + b);
      }
    };
	
    Test test_;
    hook_t<void(int, int, std::string &)> hook_;

    std::string captured_string;
    sha256::sha256_hash_type connected_id;
    uint32_t hook_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);
    auto op = hook_
                  .set_error_handler(
                      [&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                       const uint32_t &error_case_id = static_cast<uint32_t>(
                                           error_case_e::ERROR_CASE_RUNTIME)) -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    {
      auto add_result = hook_.add("FirstSumma", &test_, &Test::slot1);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("FirstSumma"), std::strlen("FirstSumma")));

      hook_(5u, 5u, captured_string);
      EXPECT_STREQ(captured_string.c_str(), "FirstSumma = 10");

      captured_string.clear();
    }

    {
      auto replace_result = hook_.replace("FirstSumma", "SecondSumma", &test_, &Test::slot2);

      EXPECT_EQ(replace_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SecondSumma"), std::strlen("SecondSumma")));

      hook_(15u, 15u, captured_string);
      EXPECT_STREQ(captured_string.c_str(), "SecondSumma = 30");
      captured_string.clear();
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
  }
}

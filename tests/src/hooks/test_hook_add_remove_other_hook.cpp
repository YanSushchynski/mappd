#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, add_remove_other_hook) {
  {
    std::string testing_string;
    std::function<void(int, int)> first_hook = [&testing_string](int a, int b) -> void {
      testing_string += std::to_string(a + b + 1);
    };

    std::function<void(int, int)> second_hook = [&testing_string](int a, int b) -> void {
      testing_string += std::to_string(a + b + 2);
    };

    hook_t<void(int, int)> hook_one;
    hook_t<void(int, int)> hook_two;

    uint32_t hook_one_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);
    auto op = hook_one
                  .set_error_handler([&hook_one_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                                          const uint32_t &error_case_id = static_cast<uint32_t>(
                                                              error_case_e::ERROR_CASE_RUNTIME)) -> void {
                    hook_one_error_id = error_id;
                  })
                  .qualifiers.at(hook_one.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    uint32_t hook_two_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);
    op = hook_two
             .set_error_handler(
                 [&hook_two_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                      const uint32_t &error_case_id = static_cast<uint32_t>(
                                          error_case_e::ERROR_CASE_RUNTIME)) -> void { hook_two_error_id = error_id; })
             .qualifiers.at(hook_two.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    {
      auto add_result = hook_one.add("FirstHook", first_hook);
      EXPECT_EQ(add_result.status.qualifiers.at(hook_one.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("FirstHook"), std::strlen("FirstHook")));
    }

    {
      auto add_result = hook_two.add("SecondHook", second_hook);
      EXPECT_EQ(add_result.status.qualifiers.at(hook_two.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SecondHook"), std::strlen("SecondHook")));
    }

    {
      auto add_result = hook_one.add("SecondHook", hook_two);
      EXPECT_EQ(add_result.status.qualifiers.at(hook_two.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SecondHook"), std::strlen("SecondHook")));
    }

    hook_one(5u, 5u);
    EXPECT_STREQ(testing_string.c_str(), (std::to_string(5u + 5u + 1u) + std::to_string(5u + 5u + 2u)).c_str());

    {
      auto remove_result = hook_one.remove("SecondHook");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_one.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SecondHook"), std::strlen("SecondHook")));
      EXPECT_EQ(remove_result.data.second, 1u);
    }

    {
      auto remove_result = hook_one.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_one.get_id()),
                static_cast<uint32_t>(hook_errno_e::HOOK_MISSING));
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
    }

    hook_one(5u, 5u);

    EXPECT_STREQ(testing_string.c_str(),
                 (std::to_string(5u + 5u + 1u) + std::to_string(5u + 5u + 2u) + std::to_string(5u + 5u + 1u)).c_str());
  }
}

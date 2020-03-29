#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, add_remove_static_members) {
  {
    class Test {
    public:
      static void slot(int a, int b, std::string &captured_string) {
        captured_string += "Tested!" + std::to_string(a + b);
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
      auto add_result = hook_.add("Summa", &Test::slot);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5, captured_string);

      EXPECT_STREQ(captured_string.c_str(), "Tested!10");

      captured_string.clear();
      connected_id = add_result.data.first;
    }

    {
      auto remove_result = hook_.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
      EXPECT_EQ(remove_result.data.first, connected_id);
      EXPECT_EQ(remove_result.data.second, 0u);
    }

    {
      auto remove_result = hook_.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_MISSING));
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
      EXPECT_EQ(remove_result.data.first, connected_id);
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
  }

  {
    class Test {
    public:
      static void slot(int a, int b, std::string &captured_string) {
        captured_string += "Tested!" + std::to_string(a + b);
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
      auto add_result = hook_.add("Summa", &Test::slot);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5, captured_string);

      EXPECT_STREQ(captured_string.c_str(), "Tested!10");

      captured_string.clear();
      connected_id = add_result.data.first;
    }

    {
      auto add_result = hook_.add("Summa", &Test::slot);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_EXISTS));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5, captured_string);

      EXPECT_STREQ(captured_string.c_str(), "Tested!10");

      captured_string.clear();
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
  }
}

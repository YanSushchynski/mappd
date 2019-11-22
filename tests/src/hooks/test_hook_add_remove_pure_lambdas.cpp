#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, add_remove_pure_lambdas) {
  {
    hook_t<void(int, int)> hook_;

    std::string captured_string;
    sha256::sha256_hash_type connected_id;

    auto func = [](int a, int b) -> void {};

    uint32_t hook_error_id = hook_errno_t::HOOK_CLEAR;

    EXPECT_EQ(hook_
                  .set_error_handler([&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                                      const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME)
                                         -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id()),
              hook_errno_t::HOOK_CLEAR);

    {
      auto add_result = hook_.add("Summa", func);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5);
      connected_id = add_result.data.first;
    }

    {
      auto remove_result = hook_.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
      EXPECT_EQ(remove_result.data.first, connected_id);
      EXPECT_EQ(remove_result.data.second, 0u);
    }

    {
      auto remove_result = hook_.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_MISSING);
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
      EXPECT_EQ(remove_result.data.first, connected_id);
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
  }

  {
    hook_t<void(int, int)> hook_;

    std::string captured_string;
    sha256::sha256_hash_type connected_id;

    auto func = [](int a, int b) -> void {};

    uint32_t hook_error_id = hook_errno_t::HOOK_CLEAR;

    EXPECT_EQ(hook_
                  .set_error_handler([&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                                      const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME)
                                         -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id()),
              hook_errno_t::HOOK_CLEAR);

    {
      auto add_result = hook_.add("Summa", func);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5);
    }

    {
      auto add_result = hook_.add("Summa", func);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_EXISTS);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5);
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
  }

  {
    hook_t<void(int, int)> hook_;

    std::string captured_string;
    sha256::sha256_hash_type connected_id;

    auto func = [](int a, int b) -> void {};

    uint32_t hook_error_id = hook_errno_t::HOOK_CLEAR;

    EXPECT_EQ(hook_
                  .set_error_handler([&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                                      const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME)
                                         -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id()),
              hook_errno_t::HOOK_CLEAR);

    {
      auto add_result = hook_.add("Summa", func);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      hook_(5, 5);

      EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
    }

    {
      auto add_result = hook_.add("Summa_new", func);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa_new"), std::strlen("Summa_new")));
      hook_(5, 5);
      EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
    }
  }
}

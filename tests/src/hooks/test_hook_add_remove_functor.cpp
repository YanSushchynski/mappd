#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Hook, add_remove_functor_t) {
  {
    hook_t<void(int32_t, int32_t)> hook_;
    std::string testing_string;
    sha256::sha256_hash_type connected_id;

    std::function<void(int32_t, int32_t)> non_pure_lambda = [&testing_string](int32_t a, int32_t b) -> void {
      testing_string = "Tested!";
    };

    functor_t functor_(non_pure_lambda);
    uint32_t hook_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);

    auto op = hook_
                  .set_error_handler(
                      [&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                       const uint32_t &error_case_id = static_cast<uint32_t>(
                                           error_case_e::ERROR_CASE_RUNTIME)) -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id());
	
    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    {
      auto add_result = hook_.add("Function", functor_);
      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Function"), std::strlen("Function")));

      hook_(5u, 5u);
      EXPECT_STREQ(testing_string.c_str(), "Tested!");

      testing_string.clear();
      connected_id = add_result.data.first;
    }

    {
      auto remove_result = hook_.remove("Function");

      EXPECT_EQ(remove_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(remove_result.data.first, connected_id);
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
  }

  {
    hook_t<void(int32_t, int32_t)> hook_;

    std::string testing_string;
    sha256::sha256_hash_type connected_id;
    std::function<void(int32_t, int32_t)> non_pure_lambda = [&testing_string](int32_t a, int32_t b) -> void {
      testing_string = "Tested!";
    };

    functor_t functor_(non_pure_lambda);
    uint32_t hook_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);

    auto op = hook_
                  .set_error_handler(
                      [&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                       const uint32_t &error_case_id = static_cast<uint32_t>(
                                           error_case_e::ERROR_CASE_RUNTIME)) -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    {
      auto add_result = hook_.add("Function", functor_);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Function"), std::strlen("Function")));

      hook_(5, 5);

      EXPECT_STREQ(testing_string.c_str(), "Tested!");

      testing_string.clear();
      connected_id = add_result.data.first;
    }

    {
      auto add_result = hook_.add("Function", functor_);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_EXISTS));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Function"), std::strlen("Function")));

      hook_(5, 5);

      EXPECT_STREQ(testing_string.c_str(), "Tested!");

      testing_string.clear();
      connected_id = add_result.data.first;
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
  }

  {
    hook_t<void(int, int)> hook_;

    std::string testing_string;
    std::function<void(int, int)> non_pure_lambda_first = [&testing_string](int32_t a, int32_t b) -> void {
      testing_string += "TestedFirst!";
    };
    std::function<void(int, int)> non_pure_lambda_second = [&testing_string](int32_t a, int32_t b) -> void {
      testing_string += "TestedSecond!";
    };

    functor_t functor_first(non_pure_lambda_first);
    functor_t functor_second(non_pure_lambda_second);
    uint32_t hook_error_id = static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR);

    auto op = hook_
                  .set_error_handler(
                      [&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                       const uint32_t &error_case_id = static_cast<uint32_t>(
                                           error_case_e::ERROR_CASE_RUNTIME)) -> void { hook_error_id = error_id; })
                  .qualifiers.at(hook_.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));

    {
      auto add_result = hook_.add("FirstFunction", functor_first);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("FirstFunction"), std::strlen("FirstFunction")));

      hook_(5, 5);

      EXPECT_STREQ(testing_string.c_str(), "TestedFirst!");
      testing_string.clear();
    }

    {
      auto add_result = hook_.add("SecondFunction", functor_second);

      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), static_cast<uint32_t>(hook_errno_e::HOOK_CLEAR));
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("SecondFunction"), std::strlen("SecondFunction")));

      hook_(5, 5);

      EXPECT_STREQ(testing_string.c_str(), "TestedFirst!TestedSecond!");
      testing_string.clear();
    }
  }
}

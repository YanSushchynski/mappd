#include "hook.hpp"
#include "gtest/gtest.h"

TEST(Hook, copy_count_check) {
  {
    class Copyable {
    public:
      Copyable(uint64_t &r_copy_counter, uint64_t &r_call_counter)
          : r_copy_counter_(r_copy_counter), r_call_counter_(r_call_counter) {
        r_copy_counter_++;
      }

      void operator()() { r_call_counter_++; };

    private:
      uint64_t &r_copy_counter_;
      uint64_t &r_call_counter_;
    };

    class Test {
    public:
      void slot_copy_check(Copyable c) { c(); }
    };

    uint64_t copy_count = 0;
    uint64_t call_count = 0;

    Copyable c(copy_count, call_count); /* +1 */
    Test test_;

    hook_t<void(Copyable)> hook_;
    uint32_t hook_error_id = hook_errno_t::HOOK_CLEAR;
    auto op = hook_
                  .set_error_handler(
                      [&hook_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                       const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        hook_error_id = error_id;
                      })
                  .qualifiers.at(hook_.get_id());

    EXPECT_EQ(op, hook_errno_t::HOOK_CLEAR);

    {
      auto add_result = hook_.add("Copy", &test_, &Test::slot_copy_check);
      EXPECT_EQ(add_result.status.qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Copy"), std::strlen("Copy")));

      for (uint32_t i = 0u; i < 10u; i++)
        hook_(c);

      EXPECT_EQ(copy_count, 1u);
      EXPECT_EQ(call_count, 10u);
    }

    EXPECT_EQ(hook_.clear().qualifiers.at(hook_.get_id()), hook_errno_t::HOOK_CLEAR);
  }
}

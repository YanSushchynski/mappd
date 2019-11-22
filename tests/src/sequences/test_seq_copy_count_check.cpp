#include "sequence.hpp"
#include "gtest/gtest.h"

TEST(Sequence, copy_count_check) {
  {
    class Copyable {
    public:
      Copyable(size_t &r_copy_counter, size_t &r_call_counter)
          : r_copy_counter_(r_copy_counter), r_call_counter_(r_call_counter) {
        r_copy_counter_++;
      }

      void operator()() { r_call_counter_++; };

    private:
      size_t &r_copy_counter_;
      size_t &r_call_counter_;
    };

    class Test {
    public:
      void slot_copy_check(Copyable c) { c(); }

      void slot_seq() {}
    };

    size_t copy_count = 0u;
    size_t call_count = 0u;
    Copyable c(copy_count, call_count); /* +1 */
    Test test_;
    sequence_t sequence_(&test_, &Test::slot_copy_check);
    uint32_t sequence_error_id = seq_errno_t::SEQ_CLEAR;

    EXPECT_EQ(sequence_
                  .set_error_handler(
                      [&sequence_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                           const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        sequence_error_id = error_id;
                      })
                  .qualifiers.at(sequence_.get_id()),
              seq_errno_t::SEQ_CLEAR);

    {
      auto add_result = sequence_.add("Copy", &test_, &Test::slot_seq);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Copy"), std::strlen("Copy")));

      for (unsigned int i = 0u; i < 10u; i++)
        sequence_(c);

      EXPECT_EQ(copy_count, 1u);
      EXPECT_EQ(call_count, 10u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }
}

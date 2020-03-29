#include "sequence.hpp"
#include "gtest/gtest.h"

TEST(Sequence, replace_by_pure_lambdas) {
  {
    class Test {
    public:
      void base(int a, int b) { (void)(a + b); }
    };

    Test test_;
    sequence_t sequence_(&test_, &Test::base);

    std::function<void(void)> sequence_part_func1 = []() -> void {};
    std::function<void(void)> sequence_part_func2 = []() -> void {};

    uint32_t sequence_error_id = seq_errno_t::SEQ_CLEAR;
    auto op = sequence_
                  .set_error_handler(
                      [&sequence_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                           const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        sequence_error_id = error_id;
                      })
                  .qualifiers.at(sequence_.get_id());

    EXPECT_EQ(op, seq_errno_t::SEQ_CLEAR);

    {
      auto add_result = sequence_.add("Seq1", sequence_part_func1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));
      sequence_(5u, 5u);
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", sequence_part_func2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));
      sequence_(5u, 5u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    std::string testing_string;

    std::function<void(int, int)> base = [](int a, int b) -> void { (void)(a + b); };
    std::function<void(void)> sequence_part_func1 = []() -> void {};
    std::function<void(void)> sequence_part_func2 = []() -> void {};
    sequence_t sequence_(base);

    uint32_t sequence_error_id = seq_errno_t::SEQ_CLEAR;
    auto op = sequence_
                  .set_error_handler(
                      [&sequence_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                           const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        sequence_error_id = error_id;
                      })
                  .qualifiers.at(sequence_.get_id());

    EXPECT_EQ(op, seq_errno_t::SEQ_CLEAR);

    {
      auto add_result = sequence_.add("Seq1", sequence_part_func1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));

      sequence_(5u, 5u);
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", sequence_part_func2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));

      sequence_(5u, 5u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    std::string testing_string;

    std::function<void(int, int)> base = [&testing_string](int a, int b) -> void {
      testing_string = std::to_string(a + b);
      (void)(a + b);
    };

    std::function<void(void)> sequence_part_func1 = []() -> void {};
    std::function<void(void)> sequence_part_func2 = []() -> void {};
    sequence_t sequence_(base);

    uint32_t sequence_error_id = seq_errno_t::SEQ_CLEAR;
    auto op = sequence_
                  .set_error_handler(
                      [&sequence_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                           const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        sequence_error_id = error_id;
                      })
                  .qualifiers.at(sequence_.get_id());

    EXPECT_EQ(op, seq_errno_t::SEQ_CLEAR);

    {
      auto add_result = sequence_.add("Seq1", sequence_part_func1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));

      sequence_(5u, 5u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u).c_str());
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", sequence_part_func2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));

      sequence_(5u, 5u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5 + 5).c_str());
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }
}

#include "sequence.hpp"
#include "gtest/gtest.h"

TEST(Sequence, replace_by_void_member_functions) {
  {
    class Test {
    public:
      int result = 0;
      void base(int a, int b) { this->result = a + b; }

      void seq1(void) { this->result++; }

      void seq2(void) { this->result += 2; }
    };

    Test test_;
    sequence_t sequence_(&test_, &Test::base);
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
      auto add_result = sequence_.add("Seq1", &test_, &Test::seq1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));

      sequence_(5u, 5u);
      EXPECT_EQ(test_.result, 5u + 5u + 1u);
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", &test_, &Test::seq2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));

      sequence_(5u, 5u);
      EXPECT_EQ(test_.result, 5u + 5u + 2u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    class Test {
    public:
      int result = 0u;
      void base(int a, int b) { this->result = a + b; }

      void seq1(void) { this->result++; }

      void seq2(void) { this->result += 2u; }
    };

    Test test_;

    std::function<void(int, int)> base = [](int a, int b) -> void {};

    sequence_t sequence_(base);

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
      auto add_result = sequence_.add("Seq1", &test_, &Test::seq1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));

      sequence_(5u, 5u);

      EXPECT_EQ(test_.result, 1u);
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", &test_, &Test::seq2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));

      sequence_(5u, 5u);

      EXPECT_EQ(test_.result, 1u + 2u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    int result = 0u;
    class Test {
    private:
      int *p_result_;
      int &r_result_;

    public:
      explicit Test(int *p_result) : p_result_(p_result), r_result_(*p_result_){};
      void base(int a, int b) { *p_result_ = a + b; }

      void seq1(void) { r_result_++; }

      void seq2(void) { r_result_ += 2u; }
    };

    Test test_(&result);
    std::string testing_string;

    std::function<void(int, int)> base = [&testing_string, &result](int a, int b) -> void {
      result = a + b;
      testing_string = std::to_string(result);
    };

    sequence_t sequence_(base);

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
      auto add_result = sequence_.add("Seq1", &test_, &Test::seq1);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first, sha256::compute(reinterpret_cast<const uint8_t *>("Seq1"), std::strlen("Seq1")));

      sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 1u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u).c_str());
    }

    {
      auto replace_result = sequence_.replace("Seq1", "Seq2", &test_, &Test::seq2);

      EXPECT_EQ(replace_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(replace_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Seq2"), std::strlen("Seq2")));

      sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 2u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u).c_str());
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }
}

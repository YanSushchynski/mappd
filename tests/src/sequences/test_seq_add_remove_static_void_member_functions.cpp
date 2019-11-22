#include "sequence.hpp"
#include "gtest/gtest.h"

TEST(Sequence, add_remove_static_void_member_functions) {
  {
    static int result = 0;
    class Test {
    public:
      void slot(int a, int b) { result = a + b; }

      static void increment(void) { result++; }
    };

    Test test_;
    sequence_t sequence_(&test_, &Test::slot);
    sha256::sha256_hash_type connected_id;
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
      auto add_result = sequence_.add("Summa", Test::increment);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      sequence_(5u, 5u);
      EXPECT_EQ(result, 5u + 5u + 1u);

      connected_id = add_result.data.first;
    }

    {
      auto remove_result = sequence_.remove("Summa");

      EXPECT_EQ(remove_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(remove_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));
      EXPECT_EQ(remove_result.data.first, connected_id);
      EXPECT_EQ(remove_result.data.second, 0u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    static int result = 0;
    class Test {
    public:
      void slot(int a, int b) { result = a + b; }

      static void increment(void) { result++; }
    };

    Test test_;
    sequence_t sequence_(&test_, &Test::slot);
    std::string testing_string;
    sha256::sha256_hash_type connected_id;

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
      auto add_result = sequence_.add("Summa", Test::increment);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 1u);

      connected_id = add_result.data.first;
    }

    {
      auto add_result = sequence_.add("Summa", Test::increment);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_EXISTS);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 1u);
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    static int result = 0;
    class Test {
    public:
      void slot(int a, int b) { result = a + b; }

      static void increment(void) { result++; }
    };

    Test test_;
    sequence_t sequence_(&test_, &Test::slot);
    std::string testing_string;
    sha256::sha256_hash_type connected_id;

    uint32_t sequence_error_id = seq_errno_t::SEQ_CLEAR;

    EXPECT_EQ(sequence_
                  .set_error_handler(
                      [&sequence_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                           const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        sequence_error_id = error_id;
                      })
                  .qualifiers.at(sequence_.get_id()),
              seq_errno_t::SEQ_CLEAR);

    for (unsigned int i = 0u; i < 10u; i++) {

      auto add_result = sequence_.add("Summa" + std::to_string(i), Test::increment);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>(("Summa" + std::to_string(i)).c_str()),
                                ("Summa" + std::to_string(i)).length()));
    }

    sequence_(5u, 5u);
    EXPECT_EQ(result, 5u + 5u + 10u);
    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }
}

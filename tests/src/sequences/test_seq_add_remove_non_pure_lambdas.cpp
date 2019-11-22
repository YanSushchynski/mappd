#include "sequence.hpp"
#include "gtest/gtest.h"

TEST(Sequence, add_remove_non_pure_lambdas) {
  {
    std::string testing_string;
    std::function<int(int, int)> base = [&testing_string](int a, int b) -> int {
      testing_string = std::to_string(a + b);
      return a + b;
    };

    std::function<int &(int &)> sequence_part_func = [&testing_string](int &a) -> int & {
      testing_string = std::to_string(a + 1u);
      a++;
      return a;
    };

    sequence_t sequence_(base);
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
      auto add_result = sequence_.add("Summa", sequence_part_func);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      int result = sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 1u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u + 1u).c_str());
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
    std::string testing_string;
    std::function<int(int, int)> base = [&testing_string](int a, int b) -> int {
      testing_string = std::to_string(a + b);
      return a + b;
    };

    std::function<int &(int &)> sequence_part_func = [&testing_string](int &a) -> int & {
      testing_string = std::to_string(a + 1u);
      a++;
      return a;
    };

    sequence_t sequence_(base);
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
      auto add_result = sequence_.add("Summa", sequence_part_func);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      int result = sequence_(5, 5);
      EXPECT_EQ(result, 5 + 5 + 1);
    }

    {
      auto add_result = sequence_.add("Summa", sequence_part_func);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_EXISTS);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>("Summa"), std::strlen("Summa")));

      int result = sequence_(5u, 5u);

      EXPECT_EQ(result, 5u + 5u + 1u);
      EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u + 1u).c_str());
    }

    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }

  {
    std::string testing_string;

    std::function<int(int, int)> base = [&testing_string](int a, int b) -> int {
      testing_string = std::to_string(a + b);
      return a + b;
    };

    std::function<int &(int &)> sequence_part_func = [&testing_string](int &a) -> int & {
      testing_string = std::to_string(a + 1u);
      a++;
      return a;
    };

    sequence_t sequence_(base);
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

      auto add_result = sequence_.add("Summa" + std::to_string(i), sequence_part_func);

      EXPECT_EQ(add_result.status.qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
      EXPECT_EQ(add_result.data.first,
                sha256::compute(reinterpret_cast<const uint8_t *>(("Summa" + std::to_string(i)).c_str()),
                                ("Summa" + std::to_string(i)).length()));
    }

    int result = sequence_(5u, 5u);

    EXPECT_EQ(result, 5u + 5u + 10u);
    EXPECT_STREQ(testing_string.c_str(), std::to_string(5u + 5u + 10u).c_str());
    EXPECT_EQ(sequence_.clear().qualifiers.at(sequence_.get_id()), seq_errno_t::SEQ_CLEAR);
  }
}

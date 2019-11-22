#include "functor.hpp"
#include "gtest/gtest.h"

static size_t copy_count = 0;
static size_t call_count = 0;

class Copyable {
public:
  Copyable() : p_copy_counter_(&copy_count), p_call_counter_(&call_count) { (*p_copy_counter_)++; }

  Copyable &operator=(Copyable const &other) { return *this; }

  void operator()() { (*p_call_counter_)++; };

private:
  size_t *p_copy_counter_;
  size_t *p_call_counter_;
};

TEST(Functor, copy_count_check) {
  {
    class Test {
    public:
      void slot_copy_check(Copyable c) { c(); }
      void slot_seq() {}
    };

    Copyable c; /* +1 */
    Test test_;
    functor_t functor_(&test_, &Test::slot_copy_check); /* +0 because return type is void, so
                                                           functor_t does not has last state */

    uint32_t functor_error_id = funct_errno_t::FUNCT_CLEAR;

    EXPECT_EQ(functor_
                  .set_error_handler(
                      [&functor_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                          const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        functor_error_id = error_id;
                      })
                  .qualifiers.at(functor_.get_id()),
              funct_errno_t::FUNCT_CLEAR);

    {
      for (unsigned int i = 0u; i < 10u; i++)
        functor_(c);

      EXPECT_EQ(copy_count, 1u);
      EXPECT_EQ(call_count, 10u);
    }

    copy_count = 0u;
    call_count = 0u;
  }

  {
    class Test {
    public:
      Copyable slot_copy_check(Copyable c) {
        c();
        return c;
      }
    };

    Copyable c; /* +1 */
    Test test_;
    functor_t functor_(&test_, &Test::slot_copy_check); /* +1 because functor_t has last state */
    uint32_t functor_error_id = funct_errno_t::FUNCT_CLEAR;

    EXPECT_EQ(functor_
                  .set_error_handler(
                      [&functor_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                          const uint32_t &error_case_id = error_case_t::ERROR_CASE_RUNTIME) -> void {
                        functor_error_id = error_id;
                      })
                  .qualifiers.at(functor_.get_id()),
              funct_errno_t::FUNCT_CLEAR);

    {
      for (unsigned int i = 0u; i < 10u; i++)
        functor_(c);

      EXPECT_EQ(copy_count, 2u);
      EXPECT_EQ(call_count, 10u);
    }

    copy_count = 0u;
    call_count = 0u;
  }
}

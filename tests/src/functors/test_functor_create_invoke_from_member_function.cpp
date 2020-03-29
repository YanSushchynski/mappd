#include "functor.hpp"
#include "gtest/gtest.h"

TEST(Functor, create_invoke_from_member_function) {
  {
    class Test {
    public:
      int sum(int a, int b) { return a + b; }
    };

    Test test_class;
    functor_t f_member(&test_class, &Test::sum);

    uint32_t functor_error_id = static_cast<uint32_t>(funct_errno_e::FUNCT_CLEAR);
    auto op = f_member
                  .set_error_handler(
                      [&functor_error_id](const sha256::sha256_hash_type &id, const uint32_t &error_id,
                                          const uint32_t &error_case_id = static_cast<uint32_t>(error_case_e::ERROR_CASE_RUNTIME)) -> void {
                        functor_error_id = error_id;
                      })
                  .qualifiers.at(f_member.get_id());

    EXPECT_EQ(op, static_cast<uint32_t>(funct_errno_e::FUNCT_CLEAR));

    int result = f_member(5u, 5u);

    EXPECT_EQ(functor_error_id, static_cast<uint32_t>(funct_errno_e::FUNCT_CLEAR));
    EXPECT_EQ(result, 5u + 5u);
  }
}

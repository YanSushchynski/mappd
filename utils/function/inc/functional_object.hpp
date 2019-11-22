#ifndef UTILS_FUNCTOR_HPP
#define UTILS_FUNCTOR_HPP

#include "function.hpp"
#include "member_function.hpp"

namespace std {
namespace function_traits {
template <typename Class> using call_operator_traits = member_function_traits<decltype(&Class::operator())>;
template <typename Class>
struct functor_traits_ : static_function_traits<typename call_operator_traits<Class>::function_t> {
  using call_operator = call_operator_traits<Class>;
};

template <typename Class> struct functor_traits : functor_traits_<remove_cvref_t<Class>> {};
} // namespace function_traits
} // namespace std

#endif /* UTILS_FUNCTOR_HPP */

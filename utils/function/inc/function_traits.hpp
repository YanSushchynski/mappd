#ifndef UTILS_FUNCTION_TRAITS_HPP
#define UTILS_FUNCTION_TRAITS_HPP

#include <functional>

#include "function.hpp"
#include "function_traits_helpers.hpp"
#include "functional_object.hpp"
#include "member_function.hpp"

namespace std {
namespace function_traits {
namespace detail {
struct function_tag {};
struct function_pointer_tag {};
struct member_function_tag {};
struct functor_tag {};

template <typename Function> struct function_traits_ : functor_traits<Function> {
  using function_category = functor_tag;
};

template <typename RetType, typename... Args>
struct function_traits_<RetType(Args...)> : static_function_traits<RetType(Args...)> {
  using function_category = function_tag;
};

template <typename RetType, typename... Args>
struct function_traits_<RetType (*)(Args...)> : static_function_traits<RetType(Args...)> {
  using function_category = function_pointer_tag;
};

template <typename Class, typename RetType, typename... Args>
struct function_traits_<RetType (Class::*)(Args...)> : member_function_traits<RetType (Class::*)(Args...)> {
  using function_category = member_function_tag;
};
} // namespace detail

template <typename Function> struct function_traits : detail::function_traits_<remove_cvref_t<Function>> {};

template <typename Function>
constexpr const std::function<const typename function_traits<Function>::function_t> to_std_function(const Function &f) {
  return std::function<typename function_traits<Function>::function_t>(f);
}

template <typename Function>
constexpr const std::function<typename function_traits<Function>::function_t>
to_std_function(typename function_traits<Function>::class_t *p_class, const Function &function) {
  using return_t = typename function_traits<Function>::return_t;
  using function_t = typename function_traits<Function>::function_t;
  return std::function<function_t>(
      [p_class, function](auto &&... args) -> return_t { return (p_class->*function)(args...); });
}

}; // namespace function_traits
}; // namespace std

#endif /* UTILS_FUNCTION_TRAITS_HPP */

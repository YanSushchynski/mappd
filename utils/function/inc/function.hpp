#ifndef UTILS_FUNCTION_HPP
#define UTILS_FUNCTION_HPP

#include "function_traits_helpers.hpp"
#include <cstddef>
#include <tuple>

namespace std {
namespace function_traits {
namespace detail {
template <typename FunctionType> struct static_function_traits_;
template <typename RetType, typename... Args> struct static_function_traits_<RetType(Args...)> {
  using return_t = RetType;
  using function_t = RetType(Args...);
  static constexpr std::uint64_t argc = types_count<Args...>::value;
  template <std::uint64_t N> using argument_t = typename types_n<N, Args...>::type;
  using args_t = std::tuple<Args...>;
  static constexpr bool is_void_v = std::is_void_v<RetType>;
};

template <typename RetType, typename... Args> const std::uint64_t static_function_traits_<RetType(Args...)>::argc;
} // namespace detail

template <typename FunctionType>
struct static_function_traits : detail::static_function_traits_<remove_cvref_t<FunctionType>> {};
template <typename FunctionType>
struct static_function_traits<FunctionType *> : detail::static_function_traits_<remove_cvref_t<FunctionType>> {};
}; // namespace function_traits
}; // namespace std

#endif /* UTILS_FUNCTION_HPP */

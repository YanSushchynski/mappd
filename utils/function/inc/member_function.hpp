#ifndef UTILS_MEMBER_FUNCTION_HPP
#define UTILS_MEMBER_FUNCTION_HPP

#include "function.hpp"
#include "function_traits_helpers.hpp"

namespace std {
namespace function_traits {
namespace detail {
struct const_tag {};
struct volatile_tag {};
struct lref_tag {};
struct rref_tag {};

template <typename Class, typename Function, typename... Qualifiers>
struct member_function_traits_qualifiers_ : static_function_traits<Function> {
  using class_t = Class;
  static constexpr bool is_const_v = types_has<const_tag, Qualifiers...>::value;
  static constexpr bool is_volatile_v = types_has<volatile_tag, Qualifiers...>::value;
  static constexpr bool is_lref_v = types_has<lref_tag, Qualifiers...>::value;
  static constexpr bool is_rref_v = types_has<rref_tag, Qualifiers...>::value;
};

template <typename Class, typename Func, typename... Qualifiers>
const bool member_function_traits_qualifiers_<Class, Func, Qualifiers...>::is_const_v;
template <typename Class, typename Func, typename... Qualifiers>
const bool member_function_traits_qualifiers_<Class, Func, Qualifiers...>::is_volatile_v;
template <typename Class, typename Func, typename... Qualifiers>
const bool member_function_traits_qualifiers_<Class, Func, Qualifiers...>::is_lref_v;
template <typename Class, typename Func, typename... Qualifiers>
const bool member_function_traits_qualifiers_<Class, Func, Qualifiers...>::is_rref_v;

template <typename MemFun> struct member_function_traits_;
template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...)>
    : member_function_traits_qualifiers_<Class, RetType(Args...)> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) volatile>
    : member_function_traits_qualifiers_<Class, RetType(Args...), volatile_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const volatile>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag, volatile_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) &>
    : member_function_traits_qualifiers_<Class, RetType(Args...), lref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const &>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag, lref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) volatile &>
    : member_function_traits_qualifiers_<Class, RetType(Args...), volatile_tag, lref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const volatile &>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag, volatile_tag, lref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) &&>
    : member_function_traits_qualifiers_<Class, RetType(Args...), rref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const &&>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag, rref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) volatile &&>
    : member_function_traits_qualifiers_<Class, RetType(Args...), volatile_tag, rref_tag> {};

template <typename Class, typename RetType, typename... Args>
struct member_function_traits_<RetType (Class::*)(Args...) const volatile &&>
    : member_function_traits_qualifiers_<Class, RetType(Args...), const_tag, volatile_tag, rref_tag> {};
}; // namespace detail

template <typename MemberFunctionPointer>
struct member_function_traits : detail::member_function_traits_<remove_cvref_t<MemberFunctionPointer>> {};
}; // namespace function_traits
}; // namespace std

#endif /* UTILS_MEMBER_FUNCTION_HPP */

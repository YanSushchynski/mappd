#ifndef UTILS_FUNCTION_HELPERS_HPP
#define UTILS_FUNCTION_HELPERS_HPP

#include <cstdint>
#include <utility>

namespace std {
namespace function_traits {
template <int N, typename... Ts> struct get;
template <int N, typename T, typename... Ts> struct get<N, std::tuple<T, Ts...>> {
  using type = typename get<N - 1, std::tuple<Ts...>>::type;
};

template <typename T, typename... Ts> struct get<0, std::tuple<T, Ts...>> { using type = T; };

template <typename T> using remove_cvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
template <typename... Types> struct types_count;
template <> struct types_count<> { static constexpr std::uint64_t value = 0; };

template <typename Type, typename... Types> struct types_count<Type, Types...> {
  static constexpr std::uint64_t value = types_count<Types...>::value + 1;
};

template <std::uint64_t n, typename... Types> struct types_n;
template <std::uint64_t N, typename Type, typename... Types>
struct types_n<N, Type, Types...> : types_n<N - 1, Types...> {};
template <typename Type, typename... Types> struct types_n<0, Type, Types...> { typedef Type type; };

template <typename Q, typename... Ts> struct types_has;
template <typename Q> struct types_has<Q> { static constexpr bool value = false; };

template <typename Q, typename... Ts> struct types_has<Q, Q, Ts...> { static constexpr bool value = true; };

template <typename Q, typename T, typename... Ts> struct types_has<Q, T, Ts...> : types_has<Q, Ts...> {};
}; // namespace function_traits
}; // namespace std

#endif /* UTILS_FUNCTION_HELPERS_HPP */

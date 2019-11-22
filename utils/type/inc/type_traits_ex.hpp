#ifndef TYPE_TRAITS_EX_HPP
#define TYPE_TRAITS_EX_HPP

#include <functional>
#include <type_traits>

namespace std {
template <typename T, typename...> struct are_same : std::true_type {};

template <typename T, typename U, typename... TT>
struct are_same<T, U, TT...> : std::integral_constant<bool, std::is_same_v<T, U> && are_same<T, TT...>{}> {};

template <typename... Args> static constexpr bool are_same_v = are_same<Args...>::value;

template <typename T, typename...> struct all_are_references : std::is_reference<T> {};

template <typename T, typename U, typename... TT>
struct all_are_references<T, U, TT...>
    : std::integral_constant<bool, std::is_reference_v<T> && std::is_reference_v<U> && all_are_references<T, TT...>{}> {
};

template <typename... Args> static constexpr bool all_are_references_v = all_are_references<Args...>::value;

template <typename T, typename...> struct all_are_lvreferences : std::is_lvalue_reference<T> {};

template <typename T, typename U, typename... TT>
struct all_are_lvreferences<T, U, TT...>
    : std::integral_constant<bool, std::is_lvalue_reference_v<T> && std::is_lvalue_reference_v<U> &&
                                       all_are_lvreferences<T, TT...>{}> {};

template <typename... Args> static constexpr bool all_are_lvreferences_v = all_are_lvreferences<Args...>::value;

template <typename T, typename...> struct all_are_rvreferences : std::is_rvalue_reference<T> {};

template <typename T, typename U, typename... TT>
struct all_are_rvreferences<T, U, TT...>
    : std::integral_constant<bool, std::is_rvalue_reference_v<T> && std::is_rvalue_reference_v<U> &&
                                       all_are_rvreferences<T, TT...>{}> {};

template <typename... Args> static constexpr bool all_are_rvreferences_v = all_are_rvreferences<Args...>::value;

template <typename T, typename...> struct all_are_pointers : std::is_pointer<T> {};

template <typename T, typename U, typename... TT>
struct all_are_pointers<T, U, TT...>
    : std::integral_constant<bool, std::is_pointer_v<T> && std::is_pointer_v<U> && all_are_pointers<T, TT...>{}> {};

template <typename... Args> static constexpr bool all_are_pointers_v = all_are_pointers<Args...>::value;

template <typename T, typename...> struct any_is_reference : std::is_reference<T> {};

template <typename T, typename U, typename... TT>
struct any_is_reference<T, U, TT...>
    : std::integral_constant<bool, std::is_reference_v<T> || std::is_reference_v<U> || any_is_reference<T, TT...>{}> {};

template <typename... Args> static constexpr bool any_is_reference_v = any_is_reference<Args...>::value;

template <typename T, typename...> struct any_is_lvreference : std::is_lvalue_reference<T> {};

template <typename T, typename U, typename... TT>
struct any_is_lvreference<T, U, TT...>
    : std::integral_constant<bool, std::is_lvalue_reference_v<T> || std::is_lvalue_reference_v<U> ||
                                       any_is_lvreference<T, TT...>{}> {};

template <typename... Args> static constexpr bool any_is_lvreference_v = any_is_lvreference<Args...>::value;

template <typename T, typename...> struct any_is_rvreference : std::is_rvalue_reference<T> {};

template <typename T, typename U, typename... TT>
struct any_is_rvreference<T, U, TT...>
    : std::integral_constant<bool, std::is_rvalue_reference_v<T> || std::is_rvalue_reference_v<U> ||
                                       any_is_rvreference<T, TT...>{}> {};

template <typename... Args> static constexpr bool any_is_rvreference_v = any_is_rvreference<Args...>::value;

template <typename T, typename...> struct any_is_pointer : std::is_pointer<T> {};

template <typename T, typename U, typename... TT>
struct any_is_pointer<T, U, TT...>
    : std::integral_constant<bool, std::is_pointer_v<T> || std::is_pointer_v<U> || any_is_pointer<T, TT...>{}> {};

template <typename... Args> static constexpr bool any_is_pointer_v = any_is_pointer<Args...>::value;

template <typename Type> struct is_function_pointer {
  static constexpr bool value =
      std::is_pointer<Type>::value ? std::is_function_v<typename std::remove_pointer_t<Type>> : false;
};

template <typename Type> static constexpr bool is_function_pointer_v = is_function_pointer<Type>::value;

template <typename T> struct is_std_function : public std::false_type {};

template <typename T> struct is_std_function<std::function<T>> : public std::true_type {};

template <typename Type> static constexpr bool is_std_function_v = is_std_function<Type>::value;

template <typename EntityType>
static constexpr bool is_entity_is_function_v =
    std::integral_constant<bool, std::is_function_pointer_v<EntityType> ||
                                     std::is_member_function_pointer_v<EntityType> ||
                                     std::is_std_function_v<EntityType> || std::is_function_v<EntityType>>::value;

template <typename Base, typename Derived, typename...> struct are_derived_from : std::is_base_of<Base, Derived> {};

template <typename Base, typename FirstDerived, typename SecondDerived, typename... Others>
struct are_derived_from<Base, FirstDerived, SecondDerived, Others...>
    : std::integral_constant<bool, std::is_base_of<Base, FirstDerived>::value &&
                                       std::is_base_of<Base, SecondDerived>::value &&
                                       are_derived_from<Base, FirstDerived, Others...>{}> {};

template <typename Base, typename... Others>
static constexpr bool are_derived_from_v = are_derived_from<Base, Others...>::value;

template <typename> struct is_tuple : std::false_type {};
template <typename... T> struct is_tuple<std::tuple<T...>> : std::true_type {};
template <typename... T> static constexpr bool is_tuple_v = is_tuple<T...>{};

template <typename S, typename T> struct is_streamable {
private:
  template <typename SS, typename TT>
  static auto test(int) -> decltype(std::declval<SS &>() << std::declval<TT>(), std::true_type());
  template <typename, typename> static auto test(...) -> std::false_type;
public:
  static const bool value = decltype(test<S, T>(0))::value;
};

template <typename S, typename T> static constexpr bool is_streamable_v = is_streamable<S, T>::value;
} // namespace std

#endif /* TYPE_TRAITS_EX_HPP */

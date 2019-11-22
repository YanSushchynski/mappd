#ifndef UTILS_TUPLE_HPP
#define UTILS_TUPLE_HPP

#include "array_traits.hpp"
#include <tuple>
#include <variant>

namespace std {
namespace {
template <int Low, int High, int Mid = (Low + High) / 2, typename = void> struct visit_at_;
template <int Low, int High, int Mid> struct visit_at_<Low, High, Mid, std::enable_if_t<(Low > High)>> {
  template <typename... T> static decltype(auto) apply_(int, T &&...) { throw std::out_of_range("visit_at"); }
};

template <int Mid> struct visit_at_<Mid, Mid, Mid> {
  template <typename Tuple, typename F> static decltype(auto) apply_(int n, F &&f, Tuple &&tp) {
    if (n != Mid)
      throw std::out_of_range("visit_at");

    return std::forward<F>(f)(std::get<Mid>(std::forward<Tuple>(tp)));
  }
};

template <int Low, int High, int Mid> struct visit_at_<Low, High, Mid, std::enable_if_t<(Low < High)>> {
  template <typename... T> static decltype(auto) apply_(int n, T &&... t) {
    if (n < Mid) {

      return visit_at_<Low, Mid - 1>::apply_(n, std::forward<T>(t)...);

    } else if (n == Mid) {

      return visit_at_<Mid, Mid>::apply_(n, std::forward<T>(t)...);

    } else {

      return visit_at_<Mid + 1, High>::apply_(n, std::forward<T>(t)...);
    }
  }
};

template <typename Array, std::size_t... Is>
static decltype(auto) array_to_tuple_(const Array &array, std::index_sequence<Is...>) {
  return std::make_tuple(array[Is]...);
}

template <typename Array, std::size_t... Is>
static decltype(auto) array_to_tuple_(const Array &&array, std::index_sequence<Is...>) {
  return std::make_tuple(array[Is]...);
}

template <typename... Args, std::size_t... I>
constexpr std::array<std::variant<Args...>, sizeof...(Args)> tuple_to_array_impl(std::tuple<Args...> const &tuple,
                                                                                 std::index_sequence<I...>) {
  using V = std::variant<Args...>;
  std::array<V, sizeof...(Args)> array = {{V(std::in_place_index_t<I>{}, std::get<I>(tuple))...}};
  return array;
}

template <class Tuple, class F, std::size_t... I>
constexpr F for_each_impl(Tuple &&t, F &&f, std::index_sequence<I...>) {
  return (void)std::initializer_list<int>{(std::forward<F>(f)(std::get<I>(std::forward<Tuple>(t))), 0)...}, f;
}

template <typename V, typename T, size_t I> auto get_getter() {
  return [](T const &t) { return V{std::in_place_index_t<I>{}, std::get<I>(t)}; };
}

template <typename Tuple, typename Indices = std::make_index_sequence<std::tuple_size<Tuple>::value>>
struct runtime_get_func_table;

template <typename Tuple, size_t... Indices> struct runtime_get_func_table<Tuple, std::index_sequence<Indices...>> {
  using return_type = typename std::tuple_element<0, Tuple>::type &;
  using get_func_ptr = return_type (*)(Tuple &) noexcept;
  static constexpr get_func_ptr table[std::tuple_size<Tuple>::value] = {&std::get<Indices>...};
};

template <typename Tuple, size_t... Indices>
constexpr typename runtime_get_func_table<Tuple, std::index_sequence<Indices...>>::get_func_ptr
    runtime_get_func_table<Tuple, std::index_sequence<Indices...>>::table[std::tuple_size<Tuple>::value];

template <typename... Args, std::size_t... I> auto tuple_getters_impl(std::index_sequence<I...>) {
  using V = std::variant<Args...>;
  using T = std::tuple<Args...>;
  using F = V (*)(T const &);
  std::array<F, sizeof...(Args)> array = {{get_getter<V, T, I>()...}};
  return array;
}

template <typename... Args> auto tuple_getters(std::tuple<Args...>) {
  return tuple_getters_impl<Args...>(std::index_sequence_for<Args...>{});
}

template <std::size_t Ofst, class Tuple, std::size_t... I>
constexpr auto slice_impl(Tuple &&t, std::index_sequence<I...>) {
  return std::forward_as_tuple(std::get<I + Ofst>(std::forward<Tuple>(t))...);
}
}; // namespace

template <std::size_t I1, std::size_t I2, class Cont> constexpr auto tuple_slice(Cont &&t) {
  static_assert(I2 >= I1, "Invalid slice");
  static_assert(std::tuple_size<std::decay_t<Cont>>::value >= I2, "Slice index out of bounds");

  return slice_impl<I1>(std::forward<Cont>(t), std::make_index_sequence<I2 - I1>{});
}

template <typename Tuple>
constexpr typename std::tuple_element<0, typename std::remove_reference<Tuple>::type>::type &runtime_get(Tuple &&t,
                                                                                                         size_t index) {
  using tuple_type = typename std::remove_reference<Tuple>::type;
  if (index >= std::tuple_size<tuple_type>::value)
    throw std::runtime_error("Out of range");
  return runtime_get_func_table<tuple_type>::table[index](t);
}

template <class Tuple, class F> constexpr F for_each(Tuple &&t, F &&f) {
  return for_each_impl(std::forward<Tuple>(t), std::forward<F>(f),
                       std::make_index_sequence<std::tuple_size<std::remove_reference_t<Tuple>>::value>{});
}

template <typename Tuple, typename Predicate> constexpr size_t find_if(Tuple &&tuple, Predicate pred) {
  uint32_t index = std::tuple_size<std::remove_reference_t<Tuple>>::value;
  uint32_t currentIndex = 0;
  bool found = false;

  for_each(tuple, [&](auto &&value) -> void {
    if (!found && pred(value)) {

      index = currentIndex;
      found = true;
    }

    ++currentIndex;
  });
  return index;
}

template <typename Tuple> struct tuple_size {
  static constexpr std::size_t size = Tuple::size;
  template <typename ConvType> constexpr operator ConvType() { return ConvType(size); }
};

template <typename Tuple, typename F> static decltype(auto) visit_at(int n, F &&f, Tuple &&tp) {
  return visit_at_<0, int(std::tuple_size<std::decay_t<Tuple>>{}) - 1>::apply_(n, std::forward<F>(f),
                                                                               std::forward<Tuple>(tp));
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{}))
array_to_tuple(const std::array<ValueType, N> &array) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{}))
array_to_tuple(const std::array<ValueType, N> &&array) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{})) array_to_tuple(const ValueType (&array)[N]) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{})) array_to_tuple(const ValueType(&&array)[N]) {
  return array_to_tuple_(array, Indices{});
}

template <typename... Args>
constexpr std::array<std::variant<Args...>, sizeof...(Args)> tuple_to_array(std::tuple<Args...> const &tuple) {
  return tuple_to_array_impl(tuple, std::index_sequence_for<Args...>{});
}

template <std::size_t, typename T> using T_ = T;
template <typename T, std::size_t... Is> auto make_tuple(std::index_sequence<Is...>) {
  return std::tuple<T_<Is, T>...>{};
}

template <typename T, std::size_t N> auto make_tuple() { return make_tuple<T>(std::make_index_sequence<N>{}); }
}; // namespace std

#endif /* UTILS_TUPLE_HPP */

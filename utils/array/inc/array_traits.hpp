#ifndef ARRAY_TRAITS_HPP
#define ARRAY_TRAITS_HPP

#include <array>

namespace std {
namespace array_traits {
namespace {
template <typename Array> struct size_;
template <typename ValueType, std::size_t N>
struct size_<std::array<ValueType, N>> {
  static constexpr std::size_t value = N;
};

template <typename ValueType, std::size_t N> struct size_<ValueType (&)[N]> {
  static constexpr std::size_t value = N;
};
}; // namespace

template <typename Array>
using element_t =
    std::remove_reference_t<decltype(*std::begin(std::declval<Array &>()))>;
template <typename Array>
static constexpr std::size_t size_v = size_<Array>::value;
}; // namespace array_traits
}; // namespace std

#endif /* ARRAY_TRAITS_HPP */

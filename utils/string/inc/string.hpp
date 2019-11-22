#ifndef STRING_HPP
#define STRING_HPP

#include <string_view>

template <char... chars> struct str : std::string_view {
  static constexpr const char value[sizeof...(chars) + 1u] = {chars..., '\0'};
  static constexpr int size = sizeof...(chars);
};

template <char... chars>
constexpr const char str<chars...>::value[sizeof...(chars) + 1u];

#endif /* STRING_HPP */

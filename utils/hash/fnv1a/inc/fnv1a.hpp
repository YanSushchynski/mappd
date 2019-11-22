#ifndef HASH_HPP
#define HASH_HPP

#include "string.hpp"
#include <cstdint>

#pragma GCC diagnostic ignored "-Wgnu-string-literal-operator-template"

constexpr uint32_t val_32_const = 0x00000000811c9dc5;
constexpr uint32_t prime_32_const = 0x0000000001000193;
constexpr uint64_t val_64_const = 0xcbf29ce484222325;
constexpr uint64_t prime_64_const = 0x00000100000001b3;

inline constexpr uint32_t hash_32_fnv1a(const char *str, const uint32_t value = val_32_const) noexcept {
  return (str[0] == '\0') ? value : hash_32_fnv1a(&str[1], (value ^ uint32_t(str[0])) * prime_32_const);
}

inline constexpr uint64_t hash_64_fnv1a(const char *str, const uint64_t value = val_64_const) noexcept {
  return (str[0] == '\0') ? value : hash_64_fnv1a(&str[1], (value ^ uint64_t(str[0])) * prime_64_const);
}

inline constexpr uint64_t sum64(const char *str, uint64_t value = 0u) noexcept {
  return (str[0u] == '\0') ? value : sum64(&str[1u], value + uint64_t(str[0u]));
}

inline constexpr uint64_t sum32(const char *str, uint32_t value = 0u) noexcept {
  return (str[0u] == '\0') ? value : sum64(&str[1u], value + uint32_t(str[0u]));
}

template <typename Char, Char... string> constexpr uint64_t operator""_h64() {
  return hash_64_fnv1a(str<string...>::value);
}

template <typename Char, Char... string> constexpr uint32_t operator""_h32() {
  return hash_32_fnv1a(str<string...>::value);
}

template <typename Char, Char... string> constexpr uint64_t operator""_s64() { return sum64(str<string...>::value); }
template <typename Char, Char... string> constexpr uint64_t operator""_s32() { return sum32(str<string...>::value); }

#endif /* HASH_HPP */

#ifndef COMPOSITION_LIST_HPP
#define COMPOSITION_LIST_HPP

#include "composition_static.hpp"

template <typename... Compositions> struct composition_list_static_t : std::tuple<Compositions...> {
private:
  static constexpr bool is_compositions = std::are_derived_from_v<composition_node_t, Compositions...>;

public:
  static constexpr uint32_t size = sizeof...(Compositions);
  static_assert(is_compositions, "Only compositions can be included to this list.");
  using base_t = std::tuple<Compositions...>;
  using this_t = composition_list_static_t<Compositions...>;

  explicit composition_list_static_t(const Compositions &... compositions) : base_t{compositions...} {};

  explicit composition_list_static_t(const this_t &) = default;
  explicit composition_list_static_t(this_t &&) = default;
  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~composition_list_static_t() = default;

  void setenv(struct env_base_t *const p_env) const {
    for (uint32_t i = 0u; i < size; i++)
      std::visit_at(
          i, [&p_env](const auto &composition) -> void { composition.setenv(p_env); }, *this);
  }

  void update_info() const {
    for (uint32_t i = 0u; i < size; i++)
      std::visit_at(
          i, [](const auto &composition) -> void { composition.update_info(); }, *this);
  }

  template <typename NameType> const composition_node_t &at(const NameType &name) const {
    const composition_node_t *p_cmp;
    const sha256::sha256_hash_type id = sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    bool found = false;
    for (uint32_t i = 0u; i < size; i++)
      std::visit_at(
          i,
          [&p_cmp, &id, &found](const auto &element) -> void {
            if (id == element.info().name_hash) {
              p_cmp = &element;
              found = true;
            }
          },
          *this);
    return (found) ? *p_cmp : throw std::out_of_range("component_at");
  }
};

#endif /* COMPOSITION_LIST_HPP */

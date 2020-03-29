#ifndef COMPONENT_LIST_HPP
#define COMPONENT_LIST_HPP

#include "component_static.hpp"

template <typename... Components> struct cmp_list_static_s : public std::tuple<Components...> {
private:
  static constexpr bool are_components = std::are_derived_from_v<cmp_base_s, Components...>;

public:
  static constexpr uint32_t size = sizeof...(Components);
  static_assert(are_components, "Only componets can be included to this list.");
  using this_t = cmp_list_static_s<Components...>;
  using base_s = std::tuple<Components...>;

  explicit cmp_list_static_s(const Components &... components) : base_s{components...} {};

  explicit cmp_list_static_s(const this_t &) = default;
  explicit cmp_list_static_s(this_t &&) = default;
  this_t &operator=(const this_t &) = delete;
  this_t &operator=(this_t &&) = delete;

  virtual ~cmp_list_static_s() = default;

  void setenv(struct env_base_s *const p_env) const {
    for (uint32_t i = 0; i < size; i++)
      std::visit_at(
          i, [&p_env](const auto &component) -> void { component.setenv(p_env); }, *this);
  }

  void update_info() const {
    for (uint32_t i = 0; i < size; i++)
      std::visit_at(
          i, [](const auto &component) -> void { component.update_info(); }, *this);
  }

  const cmp_base_s &at(const std::string &name) const {
    const cmp_base_s *p_cmp;
    const sha256::sha256_hash_type id = sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    bool found = false;

    for (unsigned int i = 0; i < size; i++)
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

#endif /* COMPONENT_LIST_HPP */

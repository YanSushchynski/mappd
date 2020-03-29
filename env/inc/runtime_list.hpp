#ifndef RUNTIME_LIST_HPP
#define RUNTIME_LIST_HPP

#include "env_utils.hpp"

template <uint32_t N>
struct runtime_list_static_s : std::array<std::tuple<sha256::sha256_hash_type, sha256::sha256_hash_type,
                                                     void (*)(int, char **, const struct env_base_s *)>,
                                          N> {
public:
  static constexpr uint32_t size = N;
  using runtime_t = void (*)(int argc, char **argv, const struct env_base_s *env_);
  using base_s = std::array<
      std::tuple<sha256::sha256_hash_type, sha256::sha256_hash_type, void (*)(int, char **, const struct env_base_s *)>,
      N>;

  explicit runtime_list_static_s(const std::tuple<std::string, std::string, runtime_t> (&runtimes)[N])
      : std::array<std::tuple<sha256::sha256_hash_type, sha256::sha256_hash_type, runtime_t>, N>() {
    for (uint32_t i = 0u; i < N; i++)
      (*this).at(i) =
          std::make_tuple(sha256::compute(reinterpret_cast<const uint8_t *>(std::get<0u>(runtimes[i]).data()),
                                          std::get<0u>(runtimes[i]).length()),
                          sha256::compute(reinterpret_cast<const uint8_t *>(std::get<1u>(runtimes[i]).data()),
                                          std::get<1u>(runtimes[i]).length()),
                          std::get<2u>(runtimes[i]));
  };

  explicit runtime_list_static_s(std::tuple<std::string, std::string, runtime_t> (&runtimes)[N])
      : std::array<std::tuple<sha256::sha256_hash_type, sha256::sha256_hash_type, runtime_t>, N>() {
    for (uint32_t i = 0u; i < N; i++)
      (*this).at(i) =
          std::make_tuple(sha256::compute(reinterpret_cast<const uint8_t *>(std::get<0u>(runtimes[i]).data()),
                                          std::get<0u>(runtimes[i]).length()),
                          sha256::compute(reinterpret_cast<const uint8_t *>(std::get<1u>(runtimes[i]).data()),
                                          std::get<1u>(runtimes[i]).length()),
                          std::get<2u>(runtimes[i]));
  };

  virtual ~runtime_list_static_s() = default;
  void operator()(int argc, char *argv[], const struct env_base_s *p_env) const {
    for (const auto &element : *this)
      element.second(argc, argv, p_env);
  }

  const std::tuple<sha256::sha256_hash_type, sha256::sha256_hash_type,
                   void (*)(int, char **, const struct env_base_s *)> &
  operator[](const uint32_t &iter) const {
    return (*static_cast<const base_s *>(this))[iter];
  }

  runtime_t operator[](const std::string &name) {
    sha256::sha256_hash_type hash = sha256::compute(reinterpret_cast<const uint8_t *>(name.data()), name.length());
    typename std::array<std::pair<sha256::sha256_hash_type, runtime_t>, N>::iterator it = std::find_if(
        this->begin(), this->end(), [&hash](const std::pair<sha256::sha256_hash_type, runtime_t> &element) -> bool {
          return element.first == hash;
        });
    return (it != this->end()) ? it->second : throw std::out_of_range(name);
  }
};

#endif /* RUNTIME_LIST_HPP */

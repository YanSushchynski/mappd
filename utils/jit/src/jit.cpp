#include "jit.hpp"

namespace jit {

static void *build_port(libconfig::Setting &port_cfg, const struct env_base_s *env);
static void *build_component(libconfig::Setting &cmp_cfg, const struct env_base_s *env);
static void *build_composition(libconfig::Setting &cmps_cfg, const struct env_base_s *env);
static void *build_static_env(libconfig::Setting &env_cfg);
static void *build_dynamic_env(libconfig::Setting &env_cfg);

void *build_env(libconfig::Setting &env_cfg) {}
} // namespace jit

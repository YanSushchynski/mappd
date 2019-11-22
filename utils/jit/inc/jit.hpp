#ifndef JIT_HPP
#define JIT_HPP

#include <dlfcn.h>
#include <unistd.h>
#include <libconfig.h++>

namespace jit {
void *build_env(libconfig::Setting &env_cfg);
} // namespace jit
#endif /* JIT_HPP */

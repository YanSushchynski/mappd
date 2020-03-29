#ifndef ENV_DYNAMIC_HPP
#define ENV_DYNAMIC_HPP

#include "env_base.hpp"

struct env_dynamic_t : public env_base_s
{
  explicit env_dynamic_t(const std::string &name) : env_base_s(name){};
  virtual void configure(const libconfig::Setting &env_config) const {};
  virtual int run(int argc, char *argv[]) const override { return 0; };
};

#endif /* ENV_DYNAMIC_HPP */

#include "codegen.hpp"
#include "env_utils.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <fmt/format.h>
#include <libconfig.h++>

std::string codegen_t::gen_port_src_code(libconfig::Setting &cfg) {}
std::string codegen_t::gen_component_src_code(libconfig::Setting &cfg){};
std::string codegen_t::gen_composition_src_code(libconfig::Setting &cfg){};
std::string codegen_t::gen_env_src_code(libconfig::Setting &cfg){};

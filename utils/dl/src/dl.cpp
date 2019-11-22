#include "dl.hpp"

dlmodule::dlmodule(libconfig::Setting &cfg) : p_handle_(jit::build_env(cfg)) {
  dlinfo(p_handle_, RTLD_DI_LINKMAP, p_link_map_);
  dlinfo(p_handle_, RTLD_DI_ORIGIN, path_);
  dlinfo(p_handle_, RTLD_DI_SERINFOSIZE, p_serinfo_);
  p_serinfo_ = static_cast<Dl_serinfo *>(std::malloc(p_serinfo_->dls_size));
  dlinfo(p_handle_, RTLD_DI_SERINFOSIZE, p_serinfo_);
  dlinfo(p_handle_, RTLD_DI_SERINFO, p_serinfo_);
};

dlmodule::~dlmodule() {
  std::free(p_serinfo_);
  dlclose(p_handle_);
};

#ifndef DL_HPP
#define DL_HPP

#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <functional>
#include <link.h>
#include <memory>
#include <libconfig.h++>
#include "jit.hpp"

struct dlmodule {
public:
  dlmodule(libconfig::Setting &cfg);
  template <typename Signature> Signature *sym(std::string &name) const {
    return static_cast<Signature *>(dlsym(p_handle_, name.c_str()));
  }

  virtual ~dlmodule();

  const inline char *path() const { return path_; }
  const inline Dl_info *info() const { return p_info_; }
  const inline Dl_serinfo *serinfo() const { return p_serinfo_; }
  const inline struct link_map *link_map() const { return p_link_map_; }

private:
  char *path_;
  Dl_info *p_info_;
  Dl_serinfo *p_serinfo_;
  void *p_handle_;
  struct link_map *p_link_map_;
};
#endif /* DL_HPP */

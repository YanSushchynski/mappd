#include <cxxabi.h>
#include <memory>
#include <string>

std::string demangle(const char *name) {
  int status = -4;
  std::unique_ptr<char, void (*)(void *)> res{
      abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
  return (status == 0) ? res.get() : name;
}

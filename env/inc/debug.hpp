#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <cstdint>

static thread_local uint32_t dbg_level;
#define DEBUG_LOG(str)                                                                                                 \
  if (dbg_level > 0) {                                                                                                 \
    std::printf("%s\r\n", str.c_str());                                                                                \
  } else {                                                                                                             \
  };

#endif /* DEBUG_HPP */

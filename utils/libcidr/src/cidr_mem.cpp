/*
 * Various libcidr memory-related functions
 */

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include "libcidr.hpp"

/* Allocate a struct cidr_addr */
CIDR *cidr_alloc(void) {
  CIDR *toret;

  toret = (CIDR *)malloc(sizeof(CIDR));
  if (toret == NULL) {
    errno = ENOMEM;
    return (NULL);
  }

  std::memset(toret, 0, sizeof(CIDR));
  return (toret);
}

/* Duplicate a CIDR */
CIDR *cidr_dup(const CIDR *src) {
  CIDR *toret;

  toret = cidr_alloc();
  if (toret == NULL)
    return (NULL); /* Preserve errno */

  std::memcpy(toret, src, sizeof(CIDR));
  return (toret);
}

/* Free a struct cidr_addr */
void cidr_free(CIDR *tofree) { free(tofree); }

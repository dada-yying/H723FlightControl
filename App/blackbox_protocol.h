/* H723 selects the faster CRC implementation; the shared SBB3 wire format
 * and F407's default implementation stay unchanged. */
#ifndef BLACKBOX_PROTOCOL_H
#define BLACKBOX_PROTOCOL_H
#include "storage/blackbox_crc.h"
#define SBB_CRC32_IMPL BlackboxCrc32
#include "../../references/blackbox_protocol/sbb3.h"
#undef SBB_CRC32_IMPL
#endif

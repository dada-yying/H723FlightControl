#ifndef BLACKBOX_DIAGNOSTIC_H
#define BLACKBOX_DIAGNOSTIC_H
#define BB_DIAGNOSTIC_FLAG 2U
#define BB_DIAGNOSTIC_FIELDS 30U
#define BB_DIAGNOSTIC_BYTES (8U + 4U * BB_DIAGNOSTIC_FIELDS)
#define BB_DIAGNOSTIC_MAGIC "BBDIAG1"
/* V3 optional block: count=0, used=128; magic[8] then 30 little-endian u32.
 * Order is shared with diagnostic() in tools/blackbox_to_csv.py. */
#endif

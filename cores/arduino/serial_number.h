#ifndef serial_number_h_
#define serial_number_h_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif
	uint32_t get_unique_id(char *buf);
	uint32_t calc_hash(char *buf, uint16_t size);
	uint32_t check_serial_number(uint32_t check_hash);
#ifdef __cplusplus
}
#endif

#endif // serial_number_h_


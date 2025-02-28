#include "serial_number.h"
#include "Arduino.h"

uint32_t get_unique_id(char *buf)
{
	enum { SERIAL_BYTE_LEN = 16 };

#ifdef __SAMD51__
	uint32_t* id_addresses[4] = {(uint32_t *)0x008061FC,(uint32_t *)0x00806010,
		(uint32_t *)0x00806014,(uint32_t *)0x00806018};
#else // samd21
	uint32_t* id_addresses[4] = {(uint32_t *)0x0080A00C,(uint32_t *)0x0080A040,
		(uint32_t *)0x0080A044,(uint32_t *)0x0080A048};

#endif

	uint8_t raw_id[SERIAL_BYTE_LEN];

	for (int i = 0; i < 4; i++) {
		for (int k = 0; k < 4; k++) {
			raw_id[4 * i + (3 - k)] = (*(id_addresses[i]) >> k * 8) & 0xff;
		}
	}

	static const char nibble_to_hex[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

	unsigned int i = 0;
	for (; i < sizeof(raw_id); i++) {
		for (int j = 0; j < 2; j++) {
			uint8_t nibble = (raw_id[i] >> (j * 4)) & 0xf;
			// Strings are UTF-16-LE encoded.
			buf[i * 2 + (1 - j)] = nibble_to_hex[nibble];
		}
	}
	buf[i*2] = 0;
	return 0;
}

uint32_t calc_hash(char *buf, uint16_t size)
{
	const uint32_t Prime = 0x01000193;
	uint32_t hash = 0x40D75A02; // seed
	for (uint32_t i = 0; i < size; i++)
	{
		hash = (buf[i] ^ hash) * Prime;
//		Serial.print(i); Serial.print(" - "); Serial.print((char)uid_buf[i]); Serial.print(" - "); Serial.println(hash, HEX);
	}
	return(hash);
}

uint32_t check_serial_number(uint32_t check_hash)
{
	char buf[65];

	get_unique_id(buf);
	uint32_t hash = calc_hash(buf, 32);
	if (hash != check_hash)
		while (1);
	return(hash);
}
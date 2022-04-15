/*
 * MD5 hash implementation and interface functions
 * Copyright (c) 2003-2005, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef MD5_HASH_H
#define MD5_HASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct MD5Context{
	uint32_t buf[4];
	uint32_t bits[2];
	uint8_t in[64];
} MD5Context_t;

void MD5Init(MD5Context_t *context);
void MD5Update(MD5Context_t *context, unsigned char const *buf, unsigned len);
void MD5Final(unsigned char digest[16], MD5Context_t *context);

#ifdef __cplusplus
}
#endif

#endif /* MD5_HASH_H */

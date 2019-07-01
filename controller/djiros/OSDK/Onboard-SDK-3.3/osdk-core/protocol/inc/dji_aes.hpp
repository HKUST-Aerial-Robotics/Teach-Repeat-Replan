/** @file AES256.hpp
 *  @version 3.3
 *  @date April 12th, 2017
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#ifndef ONBOARDSDK_AES256_H
#define ONBOARDSDK_AES256_H

#include <stdint.h>

typedef struct tagAES256Context
{
  uint8_t key[32];
  uint8_t enckey[32];
  uint8_t deckey[32];
} aes256_context;

typedef void (*ptr_aes256_codec)(aes256_context* ctx, uint8_t* buf);

uint8_t rj_xtime(uint8_t x);
void aes_subBytes(uint8_t* buf);
void aes_subBytes_inv(uint8_t* buf);
void aes_addRoundKey(uint8_t* buf, uint8_t* key);
void aes_addRoundKey_cpy(uint8_t* buf, uint8_t* key, uint8_t* cpk);
void aes_shiftRows(uint8_t* buf);
void aes_shiftRows_inv(uint8_t* buf);
void aes_mixColumns(uint8_t* buf);
void aes_mixColumns_inv(uint8_t* buf);
void aes_expandEncKey(uint8_t* k, uint8_t* rc);
void aes_expandDecKey(uint8_t* k, uint8_t* rc);
void aes256_init(aes256_context* ctx, uint8_t* k);
void aes256_done(aes256_context* ctx);
void aes256_encrypt_ecb(aes256_context* ctx, uint8_t* buf);
void aes256_decrypt_ecb(aes256_context* ctx, uint8_t* buf);

#endif // ONBOARDSDK_AES256_H

/*! @file cppforstm32.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief Support for printf to USART2 on STM32 platform
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
#ifndef CPPFORSTM32_H
#define CPPFORSTM32_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef DYNAMIC_MEMORY
void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void* pointer);
void operator delete[](void* pointer);
#endif // DYNAMIC_MEMORY

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

#endif // CPPFORSTM32_H

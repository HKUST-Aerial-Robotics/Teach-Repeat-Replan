/*! @file cppforstm32.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief Support for printf to USART2 on STM32 platform
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
 
#include "stm32f4xx.h"
#include "cppforstm32.h"
#include "BspUsart.h"

#ifdef DYNAMIC_MEMORY
void*
operator new(size_t size)
{
  if (NULL == size)
  {
#ifdef DEBUG
    printf("Error! Size is zero");
#endif // DEBUG
    return NULL;
  }
  void* p = malloc(size);
#ifdef DEBUG
  if (p == 0)
    printf("Lack Memory!");
#endif // DEBUG
  return p;
}

void*
operator new[](size_t size)
{
  return operator new(size);
}

void
operator delete(void* pointer)
{
  if (NULL != pointer)
  {
    free(pointer);
  }
}

void
operator delete[](void* pointer)
{
  operator delete(pointer);
}
#endif // DYNAMIC_MEMORY

//!@code printf link functions
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
// int fputc(int ch, FILE *f)
PUTCHAR_PROTOTYPE
{
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    ;
  USART_SendData(USART2, (uint8_t)ch);

  return (ch);
}
#ifdef __cplusplus
}
#endif //__cplusplus
//!@endcode printf link fuctions.

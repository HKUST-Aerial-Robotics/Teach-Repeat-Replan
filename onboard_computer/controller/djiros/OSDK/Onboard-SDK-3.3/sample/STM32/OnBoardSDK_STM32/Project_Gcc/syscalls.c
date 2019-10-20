#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>


#undef errno
extern int errno;

// You should define your __io_putchar
extern int __io_putchar(int ch) __attribute__((weak));

register char * stack_ptr asm("sp");


int _write(int file, char *ptr, int len)
{
  int idx;
  
  for (idx = 0; idx < len; idx++)
  {
     __io_putchar( *ptr++ );
  }
  return len;
}

caddr_t _sbrk(int incr)
{
  extern char _ebss asm("end");
  static char *heap_end;
  char *prev_heap_end;
  
  if (heap_end == 0)
    heap_end = &_ebss;
  
  prev_heap_end = heap_end;
  if (heap_end + incr > stack_ptr)
  {
    errno = ENOMEM;
    return (caddr_t) -1;
  }
  
  heap_end += incr;
  
  return (caddr_t) prev_heap_end;
}

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "OoqpVersion.h"
#include <iostream>
using namespace std;
#include <cstdio>

#ifdef WIN32
#define snprintf _snprintf
#endif

extern "C"
void printOoqpVersionString()
{
  char buff[1024];
  getOoqpVersionString( buff, 1024 );
  cout << buff << endl;
}

extern "C"
void getOoqpVersionString( char buff[], int lbuff)
{
  // One of these will be dead-code eliminated.  Do *not* convert
  // to ifdefs (so that the compiler continues to check syntax).
  if (OOQPVERSIONEXACT) {
    snprintf( buff, lbuff,
	      "OOQP Version %d.%02d.%02d - %s",
	      OOQPVERSIONMAJOR,
	      OOQPVERSIONMINOR,
	      OOQPVERSIONPATCHLEVEL,
	      OOQPVERSIONDATE );
  } else {
    snprintf( buff, lbuff,
	      "OOQP Inexact Version (based on %d.%02d.%02d - %s)",
	      OOQPVERSIONMAJOR,
	      OOQPVERSIONMINOR,
	      OOQPVERSIONPATCHLEVEL,
	      OOQPVERSIONDATE );
  }
}

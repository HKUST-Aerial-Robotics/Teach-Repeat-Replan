/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef OOQPVERSIONH 
#define OOQPVERSIONH

#define OOQPVERSIONMAJOR 0
#define OOQPVERSIONMINOR 99
#define OOQPVERSIONPATCHLEVEL 26

#define OOQPVERSIONDATE "November 1, 2014"

/* Define OOQPVERSIONEXACT to indicate a build that should be officially
   tagged by the version number -- others are development or patched builds */
#ifndef OOQPVERSIONEXACT
#define OOQPVERSIONEXACT 0
#endif

#ifdef __cplusplus
extern "C"
{
#endif
  void printOoqpVersionString();
  void getOoqpVersionString( char buff[], int lbuff);
#ifdef __cplusplus
}
#endif

#endif

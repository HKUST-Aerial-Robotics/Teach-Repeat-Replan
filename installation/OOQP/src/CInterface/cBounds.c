/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cBounds.h"
#include <stdlib.h>

void freeBounds( double ** low,  char ** ilow,
                 double ** upp,  char ** iupp )
{
  if( NULL !=  *low ) free(  *low );  *low = NULL;
  if( NULL != *ilow ) free( *ilow ); *ilow = NULL;
  if( NULL !=  *upp ) free(  *upp );  *upp = NULL;
  if( NULL != *iupp ) free( *iupp ); *iupp = NULL;

}

void newBounds( double ** low, int m, char ** ilow,
                double ** upp,        char ** iupp,
                int * ierr) 
{
  *ierr = 0;
  *low  = NULL; *ilow = NULL; *upp = NULL; *iupp = NULL;
  if( m > 0 ) {
    *low  = (double *) malloc( m * sizeof( double ) );
    *ilow = (char *)   malloc( m * sizeof( char   ) );
    *upp  = (double *) malloc( m * sizeof( double ) );
    *iupp = (char *)   malloc( m * sizeof( char   ) );
    if( NULL == *low || NULL == *ilow || NULL == *upp || NULL == *iupp ) {
      freeBounds( low, ilow, upp, iupp );
      *ierr = 1;
    }
  }
}


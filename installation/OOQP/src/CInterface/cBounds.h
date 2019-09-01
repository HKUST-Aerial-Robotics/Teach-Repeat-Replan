/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CBOUNDS
#define CBOUNDS

#ifdef __cplusplus
extern "C" {
#endif

void freeBounds( double ** low,  char ** ilow,
		 double ** upp,  char ** iupp );
void newBounds( double ** low, int m, char ** ilow,
		double ** upp,        char ** iupp,
		int * ierr) ;
#ifdef __cplusplus
}
#endif

#endif

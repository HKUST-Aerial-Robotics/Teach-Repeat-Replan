/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SUPERLUQPGENFACTORY
#define SUPERLUQPGENFACTORY

#include "QpGenSparseSeq.h"

class SuperLUQpGen : public QpGenSparseSeq {
public:
  SuperLUQpGen( int nx, int my, int mz,
		       int nnzQ, int nnzA, int nnzC );
  LinearSystem * makeLinsys( Data * prob_in );
};

#endif

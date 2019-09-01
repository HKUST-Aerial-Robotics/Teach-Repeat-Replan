/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef MA57QPGENFACTORY
#define MA57QPGENFACTORY

#include "QpGenSparseSeq.h"

class QpGenSparseMa57 : public QpGenSparseSeq {
public:
  QpGenSparseMa57( int nx, int my, int mz,
		       int nnzQ, int nnzA, int nnzC );
  LinearSystem * makeLinsys( Data * prob_in );
};

#endif

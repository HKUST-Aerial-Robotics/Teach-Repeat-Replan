/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef MA27QPGENFACTORY
#define MA27QPGENFACTORY

#include "QpGenSparseSeq.h"

class QpGenSparseMa27 : public QpGenSparseSeq {
public:
  QpGenSparseMa27( int nx, int my, int mz,
		       int nnzQ, int nnzA, int nnzC );
  LinearSystem * makeLinsys( Data * prob_in );
};

#endif

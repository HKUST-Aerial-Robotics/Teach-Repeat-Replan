/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef OOQPMONITORDATA
#define OOQPMONITORDATA

typedef struct {
  void * solver;
  void * data;
  void * vars;
  void * resids;
  int i;
  double mu;
  double rnorm;
  double dataNorm;
  int status_code;
  int level;
  double alpha;
  double sigma;
  double gap;
  void * ctx;
} OoqpMonitorData;

#ifdef __cplusplus
extern "C" {
#endif
  typedef int (*DoItCFunc) ( OoqpMonitorData * data );
#ifdef __cplusplus
}
#endif

#endif

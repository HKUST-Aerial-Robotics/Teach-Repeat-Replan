/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cstring>
#include <iostream>
using namespace std;
#include "Variables.h"
#include "Solver.h"

// default implementation for Variables::print() prints abusive
// message. Since we don't have any knowledge of how the variables are
// stored at this top level, we can't do much except print their
// dimensions.

void Variables::print()
{
  cout << " Complementary Variables = " << nComplementaryVariables << endl;
  cout << "(Cannot tell you more at this level)" << endl;
}

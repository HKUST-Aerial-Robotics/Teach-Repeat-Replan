/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

double drand(double *ix)
{
  double a, b15, b16, p, xhi, xalo, leftlo, fhi, k;
  int xhiint, leftloint, kint;

  a = 16807.0;
  b15 = 32768.0;
  b16 = 65536.0;
  p = 2147483647.0;
  xhi = *ix / b16;
  xhiint = (int) xhi;
  xhi = xhiint;
  xalo = (*ix - xhi * b16) * a;

  leftlo = xalo / b16;
  leftloint = (int) leftlo;
  leftlo = leftloint;
  fhi = xhi * a + leftlo;
  k = fhi / b15;
  kint = (int) k;
  k = kint;
  *ix = (((xalo - leftlo * b16) - p ) + (fhi - k * b15) * b16) + k;
  if(*ix < 0.0) *ix = *ix + p;

  return (*ix * 4.656612875e-10);
}

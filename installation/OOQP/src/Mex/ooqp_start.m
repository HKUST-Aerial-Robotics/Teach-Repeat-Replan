function [status, x, gamma, phi, y, z, lambda, pi] = ...
    ooqp_start( c, Q, xlow, xupp, A, dA, C, clow, cupp, ...
    x, s, y, z, v, gamma, w, phi, t, lambda, u, pi,...
    doPrint, mutol, artol )
% ooqp          Solve a convex quadratic programming problem.
%
% function [ status, x, gamma, phi, y, z, lambda, pi] = ...
%    ooqp_start( c, Q, xlow, xupp, A, dA, C, clow, cupp, ...
%    x, s, y, z, w, gamma, v, phi, t, lambda, u, pi,...
%    doPrint, mutol, artol ) )
%
%  where c, Q, xlow, xupp, A, dA, C, clow, cupp are data defining a
%  convex quadratic program (QP) defined below, and 'doPrint' is an
%  optional argument indicating the amount of progress information
%  to be printed. If 'doPrint' is 'yes' or 'on', progress information
%  is printed, while if 'doPrint' is 'no' or 'off', or if this parameter
%  is absent, no information will be printed.
%  
%  minimize:     c' * x + 0.5 * x' * Q * x
%  subject to:   A x = dA
%                clow <= C x <= cupp
%                xlow <=   x <= xupp
%
%  The matrix Q must be positive semi-definite. If it isn't, the QP is not
%  convex, and the program will give unpredictable results.
%
%  The output variable 'x' is the (primal) solution of the QP. 
%  The output variable 'status' is zero if the QP solver exits with 
%  'Optimal' status, that is, if it believes that a solution was found.
%  The remaining output variables are slack and dual variables, which
%  (if a solution is found) will satisfy the following optimality 
%  conditions (among others) to a relatively high degree of precision.
%
%  c + Q * x - A' * y - C' * z - gamma + phi = 0
%  z - lambda + pi                           = 0.

% Make empty vectors/matrices the right shape.
% c, xlow, xupp can't be empty.
n = length(c);
if( isempty(Q) ) Q = spalloc(n, n, 0); end
if( isempty(A) ) A = spalloc(0, n, 0); end
if( isempty(dA) ) dA = zeros(0,1); end
if( isempty(C)  ) C  = spalloc(0, n, 0); end
if( isempty(clow) ) clow = zeros(0,1); end
if( isempty(cupp) ) cupp = zeros(0,1); end
% x can't be empty
if( isempty(s) ) s = zeros(0,1); end
if( isempty(y) ) y = zeros(0,1); end
if( isempty(z) ) z = zeros(0,1); end
% v, gamma, w, phi can't be empty
if( isempty(t) ) t = zeros(0,1); end
if( isempty(lambda) ) lambda = zeros(0,1); end
if( isempty(u) ) u = zeros(0,1); end
if( isempty(pi) ) pi = zeros(0,1); end


ixlow = isfinite( xlow );
ixupp = isfinite( xupp );
iclow = isfinite( clow );
icupp = isfinite( cupp );

if( ~issparse(A) ) A = sparse(A); end
if( ~issparse(C) ) C = sparse(C); end;
if( ~issparse(Q) ) Q = sparse(Q); end;

xlow( ~ixlow ) = 0;
xupp( ~ixupp ) = 0;
clow( ~iclow ) = 0;
cupp( ~icupp ) = 0;

if( nargin <= 21 ) doPrint = 'no'; end
if( nargin <= 22 ) mutol   = 1e-8; end
if( nargin <= 23 ) artol   = 1e-8; end

[status, x, gamma, phi, y, z, lambda, pi] = ...
    ooqp_start_mex( c, triu(Q), xlow, ixlow, xupp, ixupp, A', dA,...
    C', clow, iclow, cupp, icupp,...
    x, s, y, z, v, gamma, w, phi, t, lambda, u, pi, doPrint, mutol, artol );


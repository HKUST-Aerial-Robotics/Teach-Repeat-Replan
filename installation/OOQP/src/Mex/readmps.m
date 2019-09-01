function [ f, c, Q, xlow, ixlow, xupp, ixupp,...
	A, dA, C, clow, iclow, cupp, icupp ] = ...
    readmps( filename )
% readmps       Read a quadratic program in quadratic MPS format.
%
% function [ f, c, Q, xlow, ixlow, xupp, ixupp,...
%	     A, dA, C, clow, iclow, cupp, icupp ] = readmps( filename )
%
% Read a convex quadratic programming problem of the form
%  minimize:     c' * x + 0.5 * x' * Q * x
%  subject to:   A x = dA
%                clow <= C x <= cupp
%                xlow <=   x <= xupp


[ f, c, Q, xlow, ixlow, xupp, ixupp,...
	A, dA, C, clow, iclow, cupp, icupp ] = readmps_mex( filename );

Q = Q + triu(Q, 1)';
A = A';
C = C';
ixlow = logical( ixlow );
ixupp = logical( ixupp );
iclow = logical( iclow );
icupp = logical( icupp );

xlow( ~ ixlow ) = -inf;
xupp( ~ ixupp ) =  inf;
clow( ~ iclow ) = -inf;
cupp( ~ icupp ) =  inf;
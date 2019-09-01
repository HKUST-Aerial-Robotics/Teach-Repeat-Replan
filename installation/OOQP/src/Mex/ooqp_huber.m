function [beta, t, gamma1, gamma2, lambda1, lambda2] =...
    ooqp_huber( X, y, cutoff)
% ooqp_huber    Huber regression 
%
% function [beta, t, gamma1, gamma2, lambda1, lambda2] =...
%    ooqp_huber( X, y, cutoff)
%
% Performs Huber regression on the data in the matrix X and the 
% observation vector y, using the parameter cutoff to define the
% switching point between the least-squares and l-1 models. The 
% solution (predictor variables) is in beta.
%
% The solution beta is chosen to minimize the following function:
%
% sum_{j=1}^{size{y}}  rho( X(j,:) * beta - y(j) ),
%
% where rho(s) is the Huber loss function defined by
%
% rho(s)  = (1/2) s^2                     if |s| <= cutoff
%         = cutoff*|s| - (1/2) cutoff^2   if |s| >  cutoff
%
% The vector beta is likely to be the only output parameter of interest
% to most users. The other returned vectors are Lagrange multupliers
% and slacks that arise in the formulation.
%
% The values returned should approximately satisfy these relationships:
%
%  t - X*beta + y + lambda2 - lambda1 = zeros(size(y))
%                              X' * t = zeros(size(beta))
%                              gamma1 =  t + cutoff*ones(size(t))
%                              gamma2 = -t + cutoff*ones(size(t))
%                   gamma1 .* lambda1 = zeros(size(gamma1))
%                   gamma2 .* lambda2 = zeros(size(gamma2))
% (lambda1, lambda2, gamma1, gamma2) >= 0.
%
error( sprintf( '%s\n%s\n',...
 'ooqp_huber is distributed as a compiled (mex) file.',...
 'The file ooqp_huber.m, (which you are trying to run)',...
 'contains only the documentation.' ) );

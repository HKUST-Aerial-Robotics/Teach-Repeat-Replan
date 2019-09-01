set I;  set J;
set QJJ within {J,J}; set CIJ within {I,J};

param objadd;  param g{J};   param Q{QJJ};
param clow{I}; param C{CIJ}; param cupp{I};
param xlow{J}; param xupp{J};

var x{j in J} >= xlow[j] <= xupp[j];

minimize total_cost: objadd + sum{j in J} g[j] * x[j] 
	 + 0.5 * sum{(j,k) in QJJ} Q[j,k] * x[j] * x[k];

subject to ineq{i in I} :
	clow[i] <= sum{(i,j) in CIJ } C[i,j] * x[j] <= cupp[i] ;

function Mat = unsparsify( ind )

sMat = sparse( ind(:,1) + 1, ind(:,2) + 1, ind(:,3 ) );
Mat = full(sMat);

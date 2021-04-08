Array = csvread('GFG.csv');
A = Array(:,2);
col2 = Array(:,3);
col3 = Array(:,4);

maxMagPos = max(A(A>0));
minMagPos = min(A(A>0));
maxMagNeg = min(A(A<0));
minMagNeg = max(A(A<0));
scatter3(A,col2,col3, 'o')
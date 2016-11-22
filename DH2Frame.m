function T = DH2Frame(alpha, theta, d, r)
%DH2Frame - output frame (transformation matrix) based on classic DH parameters.
%
% Inputs:
%    alpha, theta, d, r - Classic D-H parameters for ith link
%
% Outputs:
%    T - Transformation matrix between ith link frame to i-1th link frame
%
% Reference: https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
% Author: Longfei Zhao
% Created: 17-Nov-2016 
%% rotation matrix algorithm
Rz = @(tz)[cos(tz), -sin(tz), 0; sin(tz), cos(tz), 0; 0, 0, 1];
Ry = @(ty)[cos(ty), 0, sin(ty); 0, 1, 0; -sin(ty), 0, cos(ty)];
Rx = @(tx)[ 1, 0, 0; 0, cos(tx), -sin(tx); 0, sin(tx), cos(tx)];

% Transformation of DH frame Oi (DHi) to DH frame Oi-1 (DHprevi):
% T^(i-1)_i = TransZi-1(di)*RotZi-1(thetai)*TransXi(ri)*RotXi(alphai);
% Note that TransZi-1() is same as TransZi() function, and all DH
% parameters are with same index i.
T = [Rz(theta), [0; 0; d]; 0, 0, 0, 1] * [Rx(alpha), [r; 0; 0]; 0, 0, 0, 1];

end
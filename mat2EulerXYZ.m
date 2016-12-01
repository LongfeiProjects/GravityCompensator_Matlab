function Euler_XYZ = mat2EulerXYZ(mat)
a13 = mat(1,3); 
a21 = mat(2,1); 
a22 = mat(2,2); 
a23 = mat(2,3);
a31 = mat(3,1);
a32 = mat(3,2);
a33 = mat(3,3);

%% Euler_XYZ
tx = atan2(-a23, a33);
% ty = atan2(a13, sqrt(-a13^2+1));
% tx = atan2(-a12, a11);
ty = atan2(a13, -sin(tx)*a23 + cos(tx)*a33);
tz = atan2(cos(tx)*a21 + sin(tx)*a31, cos(tx)*a22 + sin(tx)*a32);
% [tx, ty, tz];
if (abs(a23) < 1e-6 && abs(a33) < 1e-6)
    if (ty > 0.0)
        txf = (tx + tz) / 2.0;
        tzf = (tx + tz) / 2.0;
        tyf = ty;
    else
        txf = -(tx + tz) / 2.0;
        tzf = (tx + tz) / 2.0;
        tyf = ty;
    end
else
    txf = tx;
    tzf = tz;
    tyf = ty;
end

Euler_XYZ = [txf, tyf, tzf];

end
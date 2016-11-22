function EulerXYZ = mat2EulerXYZ_AsrFrame(mat)
% code used in rtcontrol AsrFrame Class. This gives the same result as
% mat2EulerXYZ.m

% Minimum representable floating-point number in C++
DBL_MIN = 1E-37;

% Euler Angle can have two solutions with different sign
solution = 1;
if (solution == 1)
    sign = 1.0;
else
    sign = -1.0;
end

% first two ifs handle singularity, because it may be possible that
% mat(1,3) > 1.0 or < -1.0 when cos(ty) == 0
if (mat(1,3) <= (-1.0 + DBL_MIN))
    
    ty = -pi / 2.0;
    tx = 0.0; %arbitrarily chosen
    tz = atan2( mat(2,1), mat(2,2) );
    
elseif (mat(1,3) >= (1.0 - DBL_MIN ))
    
    ty = pi / 2.0;
    tx = 0.0; %arbitrarily chosen
    tz = atan2( mat(2,1), mat(2,2) );
    
else
    
    ty = atan2(mat(1, 3), sign*sqrt((mat(1, 1)*mat(1, 1)) + (mat(1, 2)*mat(1, 2))));
    c2 = cos(ty);
    tx = atan2(-mat(2, 3)/c2, mat(3, 3)/c2);
    tz = atan2(-mat(1, 2)/c2, mat(1, 1)/c2);
end

EulerXYZ = [tx, ty, tz];

end

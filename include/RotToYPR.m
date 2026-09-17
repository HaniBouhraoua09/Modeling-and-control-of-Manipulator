function [psi, theta, phi] = RotToYPR(R)
% Given a rotation matrix the function outputs the relative euler angles
% usign the convention YPR
%% Check that R is a valid rotation matrix using IsRotationMatrix().   
if ~IsRotationMatrix(R)
     error('Input R is not a valid rotation matrix.');
end
tol = 1e-3;
    
%% Compute Euler angles 
% Starting with pitch (theta)
 theta = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
% Check if (cos(theta) ? 0)
     if abs(cos(theta)) < tol
        warning('The inverse mapping is not unique. Yaw and roll are not unique.');
        psi = atan2(-R(1,2), R(2,2));
        phi = 0; % arbitrary fixing but consistent
        else
        % yaw (psi)
        psi = atan2(R(2,1), R(1,1));
        % roll (phi)
        phi = atan2(R(3,2), R(3,3));
    end
end

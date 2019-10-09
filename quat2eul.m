function [euler] = quat2eul(quaternion,axes)
    euler = euler_from_matrix(quaternion_matrix(quaternion), axes);
end

function [euler] = euler_from_matrix(matrix, axes)
    AXES2CELL.rxyz = {2, 1, 0, 1};
    AXES2CELL.rzyx = {0, 0, 0, 0};
    NEXT_AXIS = [2, 3, 1, 2];
    configs = AXES2CELL.(axes);
    firstaxis = configs{1} +1;
    parity = configs{2};
    repetition = configs{3};
    frame = configs{4};
    i = firstaxis ; 
    j = NEXT_AXIS(i+parity) ; 
    k = NEXT_AXIS(i-parity+1) ;

M = matrix(1:3, 1:3);
if repetition
    sy = sqrt(M(i, j)*M(i, j) + M(i, k)*M(i, k));
    if sy > eps
        ax = atan2( M(i, j),  M(i, k));
        ay = atan2( sy,       M(i, i));
        az = atan2( M(j, i), -M(k, i));
    else
        ax = atan2(-M(j, k),  M(j, j));
        ay = atan2( sy,       M(i, i));
        az = 0.0;
    end
else
    cy = sqrt(M(i, i)*M(i, i) + M(j, i)*M(j, i));
    if cy > eps
        ax = atan2( M(k, j),  M(k, k));
        ay = atan2(-M(k, i),  cy);
        az = atan2( M(j, i),  M(i, i));
    else
        ax = atan2(-M(j, k),  M(j, j));
        ay = atan2(-M(k, i),  cy);
        az = 0.0;
    end
end

if parity
    ax = -ax;
    ay = -ay;
    az = -az;
end
if frame
    temp = ax;
    ax = az;
    az = temp;
end
euler = [ax,ay,az];
end

function [matrix] = quaternion_matrix(quaternion)
        q_ = quaternion(:);
        nq = dot(q_, q_);
        if nq < eps
            matrix = eye(4,4);
        else
            q_ = q_*sqrt(2.0 / nq);
            q_ = q_ * q_';
            matrix = [
             1.0-q_(2, 2)-q_(3, 3),     q_(1, 2)-q_(3, 4),     q_(1, 3)+q_(2, 4), 0.0;
                q_(1, 2)+q_(3, 4), 1.0-q_(1, 1)-q_(3, 3),     q_(2, 3)-q_(1, 4), 0.0;
                q_(1, 3)-q_(2, 4),     q_(2, 3)+q_(1, 4), 1.0-q_(1, 1)-q_(2, 2), 0.0;
                            0.0,                 0.0,                 0.0, 1.0];  
        end 
    end
% %-----------------------------------------------------------------
% % Converts quaternion to Euler angles in ZYX intrinsic convention
% % Conversion adpated from :
% % https://github.com/davheld/tf/blob/master/src/tf/transformations.py
% %-----------------------------------------------------------------
% % input:
% %   normalized quaternion in format : [x, y, z, w] with ix+jy+kz+w
% %-----------------------------------------------------------------
% % return: 
% %   ai := rotation angle around x axis
% %   aj := rotation angle around y axis
% %   ak := rotation angle around z axis
% %-----------------------------------------------------------------
% 
% nq = dot(q,q);
% if nq < eps
%     m11 = 1; m21 = 1; m32 = 1; m33 = 1; m31 = 1; m23 = 1; m22 = 1;   
% else
%     q = q*sqrt(2.0/nq);
%     q = q(:);
%     q = q*q';
%     m11 = 1.0-q(2,2)-q(3,3);
%     m21 = q(1,2)+q(3,4);
%     m32 = q(2,3) + q(1,4);
%     m33 = 1.0-q(1,1)-q(2,2);
%     m31 = q(1,3) -q(2,4);
%     m23 = q(2,3) -q(1,4);
%     m22 = 1.0-q(1,1)-q(3,3);
% end
% cy = sqrt(m11*m11 + m21*m21);
% if cy > eps
%     ax = atan2( m32,  m33);
%     ay = atan2(-m31,  cy);
%     az = atan2( m21,  m11);
% else
%     ax = atan2(-m23,  m22);
%     ay = atan2(-m31,  cy);
%     az = 0.0;
% end
% euler = [ax,ay,az];


function [quaternion] = eul2quat(ai,aj,ak,axes)
    AXES2CELL.rxyz = {2, 1, 0, 1};
    AXES2CELL.rzyx = {0, 0, 0, 1};
    NEXT_AXIS = [2, 3, 1, 2];
    configs = AXES2CELL.(axes);
    firstaxis = configs{1} +1;
    parity = configs{2};
    repetition = configs{3};
    frame = configs{4};
    i = firstaxis ;
    j = NEXT_AXIS(i+parity) ;
    k = NEXT_AXIS(i-parity+1) ;
    if frame
        temp = ai;
        ai = ak;
        ak = temp;
    end
    if parity
        aj = -aj;
    end
    ai = ai/2.0;
    aj = aj/2.0;
    ak = ak/2.0;
    ci = cos(ai);
    si = sin(ai);
    cj = cos(aj);
    sj = sin(aj);
    ck = cos(ak);
    sk = sin(ak);
    cc = ci*ck;
    cs = ci*sk;
    sc = si*ck;
    ss = si*sk;

    quaternion = zeros(4,1);
    if repetition
        quaternion(i) = cj*(cs + sc);
        quaternion(j) = sj*(cc + ss);
        quaternion(k) = sj*(cs - sc);
        quaternion(4) = cj*(cc - ss);
    else
        quaternion(i) = cj*sc - sj*cs;
        quaternion(j) = cj*ss + sj*cc;
        quaternion(k) = cj*cs - sj*sc;
        quaternion(4) = cj*cc + sj*ss;
    end
    if parity
        quaternion(j) = -1*quaternion(j);
    end
% %-----------------------------------------------------------------
% % Converts Euler angles in ZYX intrinsic convention to Quaternions.
% % Conversion Formulas are derived from
% % https://github.com/davheld/tf/blob/master/src/tf/transformations.py
% %-----------------------------------------------------------------
% % input:
% %   ai := rotation angle around x axis
% %   aj := rotation angle around y axis
% %   ak := rotation angle around z axis
% %-----------------------------------------------------------------
% % return: normalized quaternion in format : [x, y, z, w] with ix+jy+kz+w
% %-----------------------------------------------------------------
%     
% %     bug_free_guarantee = ak;
% %     ak = ai;
% %     ai = bug_free_guarantee;
%    
%  
%     ai = ai/2.0;
%     aj = aj/2.0;
%     ak = ak/2.0;
%     ci = cos(ai);
%     si = sin(ai);
%     cj = cos(aj);
%     sj = sin(aj);
%     ck = cos(ak);
%     sk = sin(ak);
%     cc = ci*ck;
%     cs = ci*sk;
%     sc = si*ck;
%     ss = si*sk;
%     
%     quaternion = zeros(4,1);
%     quaternion(1) = cj*sc - sj*cs;
%     quaternion(2) = cj*ss + sj*cc;
%     quaternion(3) = cj*cs - sj*sc;
%     quaternion(4) = cj*cc + sj*ss;
end


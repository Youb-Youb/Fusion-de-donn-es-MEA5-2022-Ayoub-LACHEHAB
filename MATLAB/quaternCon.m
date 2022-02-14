function qc = quaternCon(q)
%  q = quat_conj(q)
%  Conjugate quaternion
%
%   Input arguments:
%   q -  Attitude quaternion [1,4]
%
%   Output arguments:
%   qconj -  Conjugate attitude quaternion [1,4]
%% Compute conjugate quaternion
    qc = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];

end
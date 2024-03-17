function EulerAngles = WrapperQuat2Angle(q)

[psi,theta,phi] = quat2angle(q');

EulerAngles = [phi;theta;psi];
end


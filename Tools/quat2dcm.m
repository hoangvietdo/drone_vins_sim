function dcm = quat2dcm(quat)
    %Explanation: This function converts from quat to DCM
    %Input: quat[4x1]
    %Output : dcm[3x3]
    %Reference: Equation 3.59 - Straodown inertial Navigation Technology

a = quat(1, 1);
b = quat(2, 1);
c = quat(3, 1);
d = quat(4, 1);

dcm = [(a^2 + b^2 - c^2 - d^2), 2 * (b * c - a * d), 2 * (b * d + a * c);
        2*(b * c + a * d), (a^2 - b^2 + c^2 - d^2), 2 * (c * d - a * b);
        2 * (b * d - a * c), 2 * (c * d + a * b), (a^2 - b^2 - c^2 + d^2)];
end
function dcm = euler2dcm(euler)
    %Explanation: This function converts from euler to dcm
    %Input: euler[3x1]
    %Output: dcm[3x3]
    %Reference Equation 3.63 - Strapdown inertial navigation

phi = euler(1, 1);
theta = euler(2, 1);
psi = euler(3, 1);

dcm = [cos(theta) * cos(psi), -cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi), sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);
       cos(theta) * sin(psi), cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi), -sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);
       -sin(theta), sin(phi) * cos(theta), cos(phi) * cos(theta)];

end
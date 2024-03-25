function quat = euler2quat(euler)
    %Explanation: This function converts from euler to quat
    %Input: euler[3x1]
    %Output: quat[4x1]
    %Reference:Equation 3.65 - Strapdown inertial navigation

 phi = euler(1, 1);
 theta = euler(2, 1);
 psi = euler(3, 1);

 quat = [cos(phi / 2) * cos(theta / 2) * cos(psi / 2) + sin(phi / 2) * sin(theta / 2) * sin(psi / 2);
         sin(phi / 2) * cos(theta / 2) * cos(psi / 2) - cos(phi / 2) * sin(theta / 2) * sin(psi / 2);
         cos(phi / 2) * sin(theta / 2) * cos(psi / 2) + sin(phi / 2) * cos(theta / 2) * sin(psi / 2);
         cos(phi / 2) * cos(theta / 2) * sin(psi / 2) + sin(phi / 2) * sin(theta / 2) * cos(psi / 2)];
 quat = quatNormalize(quat);
end
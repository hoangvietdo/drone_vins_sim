function quat = dcm2quat(dcm)
    %Explanation: This function converts from dcm to quat
    %Input: dcm[3x3]
    %Output: quat[4x1]
    %Reference: Equation 3.64 -Strapdown Inertial Navigation

 a = 1/2 * (1 + dcm(1, 1) + dcm(2, 2) + dcm(3, 3)).^(1/2);
 b = (1 / (4 * a)) * (dcm(3, 2) - dcm(2, 3));
 c = (1 / (4 * a)) * (dcm(1, 3) - dcm(3, 1));
 d = (1 / (4 * a)) * (dcm(2, 1) - dcm(1, 2));

 quat = [a; b; c; d];
 quat = quatNormalize(quat);

end
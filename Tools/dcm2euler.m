function euler = dcm2euler(dcm)
    % Explanation: This function converts from Direction Cosine Matrix (DCM) to
    %                     Euler Angle 321 sequence
    % Input:  A [3x3] DCM Cbn
    % Output: 3 Euler angles = Roll (phi), Pitch (theta), Yaw (psi)
    % Reference: Equation 3.66 - Strapdown Inertial Navigation Technology - David H. Titterton and John L. Weston

    phi = atan(dcm(3,2)/dcm(3,3));
    theta = asin(-dcm(3,1));
    psi = atan(dcm(2,1)/dcm(1,1));

    euler = [phi; theta; psi];
end
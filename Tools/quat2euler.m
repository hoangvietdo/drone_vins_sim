function euler = quat2euler(quat)
    %Explanation: This function converts from quat to euler
    %Input: quat[4x1]
    %Output: euler[3x1]

    
dcm = quat2dcm(quat);
euler = dcm2euler(dcm);


end
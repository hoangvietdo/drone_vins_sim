function quatNorm = quatNormalize(quat)
    % Normalize quaternion to make sure norm 2 of quaternion |q|_2 = 1
    quatNorm = quat ./ norm(quat);
end
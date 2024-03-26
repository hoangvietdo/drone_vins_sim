function matrix = skewSymmetricMatrix(vector)
    % Explanation: This function converts a random [3x1] vector into a skew-symmetric matrix [3x3]
    % Input:  A [3x1] vector
    % Output: A [3x3] Matrix
    % Reference: https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    a1 = vector(1, 1);
    a2 = vector(2, 1);
    a3 = vector(3, 1);

    matrix = [0  -a3  a2;
              a3  0  -a1;
             -a2  a1  0];
end
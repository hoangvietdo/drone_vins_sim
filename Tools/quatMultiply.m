function result = quatMultiply(q, p)
    % Explanation: This function multiply a quaternion q and quaternion p
    % Input:  2 Quaternion vector [4x1];        
    % Output: Quaternion [4x1] Matrix
    % Strapdown - Page: 43 equation 3.56

    
    a = q(1,1);
    b = q(2,1);
    c = q(3,1);
    d = q(4,1);

    e = p(1,1);
    f = p(2,1);
    g = p(3,1);
    h = p(4,1);

    result = [a -b -c -d;
              b  a -d c;
              c  d  a -b;
              d -c  b  a] * [e;f;g;h];
    result = quatNormalize(result);

end
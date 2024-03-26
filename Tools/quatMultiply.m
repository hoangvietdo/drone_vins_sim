% The MIT License
% 
% Copyright (c) 2021 Hoang Viet Do
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

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
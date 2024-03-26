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
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
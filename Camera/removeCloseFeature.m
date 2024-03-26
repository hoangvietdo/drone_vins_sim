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

function temp = removeCloseFeature(feature, distance)
    % A stupid way to remove feature that are too close to each other
    n = 1;
    temp = [];
    temp(:, 1) = feature(:, 1);

    for i = 1:1:length(feature)
        count = 0;
        for j = i + 1 : 1 : length(feature)
            if norm(feature(:, i) - feature(:, j))  <= distance
                count = 1;
            end
        end
        if count == 0
            temp(:, n) = feature(:, i);
            n = n + 1;
        end
    end
end
function [ projected, valid ] = projectPoints( points, K, T, D, imageSize, sortPoints )
%PROJECTPOINTS Projects 3d points onto a plane using a camera model.
%   Applies a standard pin-point camera model with optional distortion
%   parameters to the points.
%--------------------------------------------------------------------------
%   Required Inputs:
%--------------------------------------------------------------------------
%   points- nxm set of 3D points, the first 3 columns are the point (x,y,z)
%       locations and the remaining columns the intensity values 
%       (i1,i2,etc)
%   K- either a 3x3 or 3x4 camera matrix
%
%--------------------------------------------------------------------------
%   Optional Inputs: (will give default values on empty array [])
%--------------------------------------------------------------------------
%   T- 4x4 transformation matrix to apply to points before projecting them
%       (default identity matrix)
%   D- 1xq distortion vector that matches opencv's implementation 
%       [k1,k2,p1,p2,k3], q can be from 1 to 5 and any values not given are
%       set to 0 (default [0,0,0,0,0])
%   imageSize- 1x2 vector that gives the size of image (y,x) the points
%       will be projected onto, if it is given points that are outside this
%       size or negitive will be treated as invalid (default [])
%   sortPoints- 1x1 binary value, if true points are sorted by their
%       distance (furthest away first) from the camera (useful for 
%       rendering points), (default false)
%
%--------------------------------------------------------------------------
%   Outputs:
%--------------------------------------------------------------------------
%   projected- nx(m-1) set of 2D points, the first 2 columns give the
%       points (x,y) location and the remaining columns the intensity
%       values (i1,i2,etc). Points behind the camera are assigned nan as
%       their position.
%   valid- nx1 binary vector, true if a 2d point is in front of the camera
%       and projects onto a plane of size imageSize.
%
%--------------------------------------------------------------------------
%   References:
%--------------------------------------------------------------------------
%   The equations used in this implementation were taken from
%   http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
%   http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html

%% check inputs

validateattributes(points, {'numeric'},{'2d'});
if(size(points,2) < 3)
    error('points must have at least 3 columns, currently has %i',size(points,2));
end
points = double(points);

if(size(K,2) == 3)
    K(end,4) = 0;
end
validateattributes(K, {'numeric'},{'size',[3,4]});
K = double(K);

if(nargin < 3)
    T = [];
end
if(isempty(T))
    T = eye(4);
else
    validateattributes(T, {'numeric'},{'size',[4,4]});
end
T = double(T);

if(nargin < 4)
    D = [];
end
if(isempty(D))
    D = [0,0,0,0,0];
else
    validateattributes(D, {'numeric'},{'nrows',1});
end
if(size(D,2) > 5)
    error('distortion vector D, must have 5 or less columns currently has %i',size(D,2));
end
D = double(D);

if(nargin < 5)
    imageSize = [];
end
if(~isempty(imageSize))
    validateattributes(imageSize, {'numeric'},{'size',[1,2],'positive'});
end

if(nargin < 6)
    sortPoints = [];
end
if(isempty(sortPoints))
    sortPoints = false;
else
    validateattributes(sortPoints, {'logical'},{'scalar'});
end

%% project points

%split distortion into radial and tangential
if(size(D,2) < 5)
    D(1,5) = 0;
end
k = [D(1),D(2),D(5)];
p = [D(3),D(4)];

%split into point locations and colour
if(size(points,2) > 3)
    colour = points(:,4:end);
    points = points(:,1:3);
else
    colour = zeros(size(points,1),0);
end

%transform points
points = (T*[points,ones(size(points,1),1)]')';

% Make the points ideal
% points(:,3) = 3;
%
if(sortPoints)
    %sort points by distance from camera
    dist = sum(points(:,1:3).^2,2);
    [~,idx] = sort(dist,'descend');
    points = points(idx,:);
    colour = colour(idx,:);
end

%reject points behind camera
valid = points(:,3) > 0;
points = points(valid,:);
colour = colour(valid,:);

%project onto a plane using normalized image coordinates
x = points(:,1)./points(:,3);
y = points(:,2)./points(:,3);

%find radial distance
r2 = x.^2 + y.^2;

%find tangential distortion
xTD = 2*p(1)*x.*y + p(2).*(r2 + 2*x.^2);
yTD = p(1)*(r2 + 2*y.^2) + 2*p(2)*x.*y;

%find radial distortion
xRD = x.*(1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3); 
yRD = y.*(1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3); 

%combine distorted points
x = xRD + xTD;
y = yRD + yTD;

%project distorted points back into 3D
points = [x,y,ones(size(x,1),1)].*repmat(points(:,3),1,3);

%project using camera matrix
points = (K*[points, ones(size(points,1),1)]')';
points = points(:,1:2)./repmat(points(:,3),1,2);

%output nans for points behind camera
projected = nan(size(valid,1),2+size(colour,2));
projected(valid,:) = [points,colour];

%set points outside of the image region as invalid
if(~isempty(imageSize))
    inside = points(:,1) < imageSize(1);
    inside = and(inside, points(:,2) < imageSize(2));
    inside = and(inside, points(:,1) >= 0);
    inside = and(inside, points(:,2) >= 0);
    valid(valid) = inside;
end
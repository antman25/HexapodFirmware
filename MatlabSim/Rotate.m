function [ output ] = Rotate( input_point, rot_values)
%HEXBODY Summary of this function goes here
%   Detailed explanation goes here
x_rot_rad = degtorad(rot_values(1,1));
y_rot_rad = degtorad(rot_values(2,1));
z_rot_rad = degtorad(rot_values(3,1));

Rx = [1 0 0; 0 cos(x_rot_rad) -sin(x_rot_rad); 0 sin(x_rot_rad) cos(x_rot_rad)];
Ry = [cos(y_rot_rad) 0 sin(y_rot_rad); 0 1 0; -sin(y_rot_rad) 0 cos(y_rot_rad)];
Rz = [cos(z_rot_rad) -sin(z_rot_rad) 0; sin(z_rot_rad) cos(z_rot_rad) 0; 0 0 1];

rot_matrix = Rx*Rz*Ry;

output = rot_matrix * input_point;
end


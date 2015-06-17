function [ output ] = RotateTranslate( input_point, body_rot, body_translate)
%HEXBODY Summary of this function goes here
%   Detailed explanation goes here
body_x_rot_rad = degtorad(body_rot(1,1));
body_y_rot_rad = degtorad(body_rot(2,1));
body_z_rot_rad = degtorad(body_rot(3,1));

Rx = [1 0 0; 0 cos(body_x_rot_rad) -sin(body_x_rot_rad); 0 sin(body_x_rot_rad) cos(body_x_rot_rad)];
Ry = [cos(body_y_rot_rad) 0 sin(body_y_rot_rad); 0 1 0; -sin(body_y_rot_rad) 0 cos(body_y_rot_rad)];
Rz = [cos(body_z_rot_rad) -sin(body_z_rot_rad) 0; sin(body_z_rot_rad) cos(body_z_rot_rad) 0; 0 0 1];

rot_matrix = Rx*Rz*Ry;

output = body_translate + rot_matrix * input_point;
end


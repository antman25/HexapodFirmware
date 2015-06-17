function [ output ] = TransformCoord( input_point, beta )

%Ry = [cos(degtorad(beta)) 0 sin(degtorad(beta)); 0 1 0; -sin(degtorad(beta)) 0 cos(degtorad(beta))];
%Ry_prime = Ry.';
Rz = [cos(degtorad(beta)) -sin(degtorad(beta)) 0; sin(degtorad(beta)) cos(degtorad(beta)) 0; 0 0 1];
Rz_prime = Rz.';
output = Rz_prime * input_point;

end


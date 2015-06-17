function [ CoxaAngle,FemurAngle,TibiaAngle ] = LegIK( body_delta, foot_delta )
%LEGIK Summary of this function goes here
%   Detailed explanation goes here
CoxaLength = 29;
FemurLength = 76;
TibiaLength = 106;

body_x = body_delta(1,1);
body_y = body_delta(2,1);
body_z = body_delta(3,1);

foot_x = (CoxaLength + FemurLength) + foot_delta(1,1) - body_x;
foot_y = foot_delta(2,1) - body_y;
foot_z = TibiaLength + foot_delta(3,1) - body_z;


%L1 = foot_x - body_x;
%Zoffset = body_z - foot_z;

L = sqrt( foot_z^2 + (foot_x - CoxaLength)^2);
A1 = acos( foot_z / L );
A2 = acos( (L^2 + FemurLength^2 - TibiaLength^2) / (2*L*FemurLength) );
A = radtodeg(A1 + A2);

B = acos( (FemurLength^2 + TibiaLength^2 - L^2) / (2*FemurLength*TibiaLength) );

CoxaAngle = radtodeg(atan2( foot_y  ,foot_x  ));
FemurAngle = A - 90;
TibiaAngle = 90 - radtodeg(B);

%CoxaAngle = AngleToPWM(CoxaAngle);
%FemurAngle = AngleToPWM(FemurAngle);
%TibiaAngle = AngleToPWM(TibiaAngle);


end
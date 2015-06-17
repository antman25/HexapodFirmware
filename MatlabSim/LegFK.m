function [ CoxaPt,FemurPt,FootPt ] = LegFK( BodyPt, BodyAng, CoxaAngle, FemurAngle, TibiaAngle )
%LEGFK Summary of this function goes here
%   Detailed explanation goes here
CoxaLength = 29;
FemurLength = 76;
TibiaLength = 106;

Rfemur = [cos(degtorad(FemurAngle)) 0 sin(degtorad(FemurAngle)); 0 1 0; -sin(degtorad(FemurAngle)) 0 cos(degtorad(FemurAngle))];
Rfoot = [cos(degtorad(TibiaAngle)) 0 sin(degtorad(TibiaAngle)); 0 1 0; -sin(degtorad(TibiaAngle)) 0 cos(degtorad(TibiaAngle))];


CoxaPt = [CoxaLength;0;0];
FemurPt_i = Rfemur * [FemurLength;0;0];
FootPt_i = Rfoot * [0;0;-TibiaLength];

FemurPt = CoxaPt + FemurPt_i;
FootPt = FemurPt + FootPt_i;

beta = degtorad(BodyAng + CoxaAngle);
Rz = [cos(beta) -sin(beta) 0; sin(beta) cos(beta) 0; 0 0 1];

CoxaPt = BodyPt + Rz * CoxaPt;
FemurPt = BodyPt + Rz * FemurPt;
FootPt =  BodyPt + Rz * FootPt;
end


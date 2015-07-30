function [ CoxaPt,FemurPt,FootPt ] = LegFK( BodyPt, BodyAng, CoxaAngle, FemurAngle, TibiaAngle )
%LEGFK Summary of this function goes here
%   Detailed explanation goes here
CoxaLength = 29;
FemurLength = 76;
TibiaLength = 106;

plot_femur_angle = FemurAngle;
plot_tibia_angle = TibiaAngle;

Rfemur = [cos(degtorad(plot_femur_angle)) 0 sin(degtorad(plot_femur_angle)); 0 1 0; -sin(degtorad(plot_femur_angle)) 0 cos(degtorad(plot_femur_angle))];
Rfoot = [cos(degtorad(plot_tibia_angle)) 0 sin(degtorad(plot_tibia_angle)); 0 1 0; -sin(degtorad(plot_tibia_angle)) 0 cos(degtorad(plot_tibia_angle))];


CoxaPt = [CoxaLength;0;0];
FemurPt_i = Rfemur * [FemurLength;0;0];
FootPt_i = Rfoot * (Rfemur*[0;0;-TibiaLength]);

FemurPt = CoxaPt + FemurPt_i;
FootPt = FemurPt + FootPt_i;

plot_coxa_angle = degtorad(BodyAng + CoxaAngle);
Rcoxa = [cos(plot_coxa_angle) -sin(plot_coxa_angle) 0; sin(plot_coxa_angle) cos(plot_coxa_angle) 0; 0 0 1];

CoxaPt = BodyPt + Rcoxa * CoxaPt;
FemurPt = BodyPt + Rcoxa * FemurPt;
FootPt =  BodyPt + Rcoxa * FootPt;
end


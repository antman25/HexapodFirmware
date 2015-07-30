function [  ] = PlotHexapod( body_translate, body_rotate, RF_gait_translate, RF_gait_rot, RM_gait_translate, RM_gait_rot, RR_gait_translate, RR_gait_rot, LF_gait_translate, LF_gait_rot, LM_gait_translate, LM_gait_rot, LR_gait_translate, LR_gait_rot, Rcolor, Lcolor)

CoxaLength = 29.0;
FemurLength = 76.0;
TibiaLength = 106.0;

RF_body_init = [ 43;  82; 0 ];
RM_body_init = [ 63;   0; 0 ];
RR_body_init = [ 43; -82; 0 ];
LF_body_init = [-43;  82; 0 ];
LM_body_init = [-63;   0; 0 ];
LR_body_init = [-43; -82; 0 ];

LegInitXYCos60 = (CoxaLength + FemurLength) * cos(degtorad(60));
LegInitXYSin60 = (CoxaLength + FemurLength) * sin(degtorad(60));
LegInitX = (CoxaLength + FemurLength);

RF_foot_init = [LegInitXYCos60; LegInitXYSin60; TibiaLength];
RM_foot_init = [LegInitX; 0; TibiaLength];
RR_foot_init = [LegInitXYCos60; -LegInitXYSin60; TibiaLength];
LF_foot_init = [-LegInitXYCos60; LegInitXYSin60; TibiaLength];
LM_foot_init = [-LegInitX; 0; TibiaLength];
LR_foot_init = [-LegInitXYCos60; -LegInitXYSin60; TibiaLength];

RF_body_diff = RF_body_init - Rotate( RF_body_init, body_rotate) + body_translate;
RM_body_diff = RM_body_init - Rotate( RM_body_init, body_rotate) + body_translate;
RR_body_diff = RR_body_init - Rotate( RR_body_init, body_rotate) + body_translate;
LF_body_diff = LF_body_init - Rotate( LF_body_init, body_rotate) + body_translate;
LM_body_diff = LM_body_init - Rotate( LM_body_init, body_rotate) + body_translate;
LR_body_diff = LR_body_init - Rotate( LR_body_init, body_rotate) + body_translate;

RF_foot_diff = Rotate(RF_body_init + RF_foot_init, RF_gait_rot) - (RF_body_init + RF_foot_init) + RF_gait_translate;
RM_foot_diff = Rotate(RM_body_init + RM_foot_init, RM_gait_rot) - (RM_body_init + RM_foot_init) + RM_gait_translate;
RR_foot_diff = Rotate(RR_body_init + RR_foot_init, RR_gait_rot) - (RR_body_init + RR_foot_init) + RR_gait_translate;
LF_foot_diff = Rotate(LF_body_init + LF_foot_init, LF_gait_rot) - (LF_body_init + LF_foot_init) + LF_gait_translate;
LM_foot_diff = Rotate(LM_body_init + LM_foot_init, LM_gait_rot) - (LM_body_init + LM_foot_init) + LM_gait_translate;
LR_foot_diff = Rotate(LR_body_init + LR_foot_init, LR_gait_rot) - (LR_body_init + LR_foot_init) + LR_gait_translate;

RF_diff_ik = TransformCoord(RF_body_diff + RF_foot_diff, 60);
RM_diff_ik = TransformCoord(RM_body_diff + RM_foot_diff, 0);
RR_diff_ik = TransformCoord(RR_body_diff + RR_foot_diff, 300);
LF_diff_ik = TransformCoord(LF_body_diff + LF_foot_diff, 120);
LM_diff_ik = TransformCoord(LM_body_diff + LM_foot_diff, 180);
LR_diff_ik = TransformCoord(LR_body_diff + LR_foot_diff, 240);

hold on;
xlim([-300 300]);
ylim([-300 300]);
zlim([-300 300]);
view(30,30);

%PlotLeg(RF_body_init, RF_diff_ik, 60, 'b');
%PlotLeg(RM_body_init, RM_diff_ik, 0, 'b');
%PlotLeg(RR_body_init, RR_diff_ik, 300, 'b');
%PlotLeg(LF_body_init, LF_diff_ik, 120, 'b');
%PlotLeg(LM_body_init, LM_diff_ik, 180, 'b');
%PlotLeg(LR_body_init, LR_diff_ik, 240, 'b');


PlotLeg(RF_body_init, RF_diff_ik, 60, Rcolor);
PlotLeg(RM_body_init, RM_diff_ik, 0, Rcolor);
PlotLeg(RR_body_init, RR_diff_ik, 300, Rcolor);
PlotLeg(LF_body_init, LF_diff_ik, 120, Lcolor);
PlotLeg(LM_body_init, LM_diff_ik, 180, Lcolor);
PlotLeg(LR_body_init, LR_diff_ik, 240, Lcolor);


end


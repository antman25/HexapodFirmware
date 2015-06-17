function [  ] = TestDrawHex( body_rot, body_translate, RF_foot, RM_foot, RR_foot, LF_foot, LM_foot, LR_foot, Rcolor, Lcolor )

RF_body_init = [ 43;  82; 0 ];
RM_body_init = [ 63;   0; 0 ];
RR_body_init = [ 43; -82; 0 ];
LF_body_init = [-43;  82; 0 ];
LM_body_init = [-63;   0; 0 ];
LR_body_init = [-43; -82; 0 ];


hold on;

xlim([-300 300]);
ylim([-300 300]);
zlim([-300 300]);
grid on;


view(30,30);

xlabel('X');
ylabel('Y');
zlabel('Z');

RF_point = RotateTranslate(RF_body_init, body_rot, body_translate);
RM_point = RotateTranslate(RM_body_init, body_rot, body_translate);
RR_point = RotateTranslate(RR_body_init, body_rot, body_translate);

LF_point = RotateTranslate(LF_body_init, body_rot, body_translate);
LM_point = RotateTranslate(LM_body_init, body_rot, body_translate);
LR_point = RotateTranslate(LR_body_init, body_rot, body_translate);

RF_diff = RF_point - RF_body_init;
RM_diff = RM_point - RM_body_init;
RR_diff = RR_point - RR_body_init;

LF_diff = LF_point - LF_body_init;
LM_diff = LM_point - LM_body_init;
LR_diff = LR_point - LR_body_init;

PlotLeg( RF_body_init, RF_diff, RF_foot, 60, Rcolor );
PlotLeg( RM_body_init, RM_diff, RM_foot, 0, Rcolor );
PlotLeg( RR_body_init, RR_diff, RR_foot, 300, Rcolor );

PlotLeg( LF_body_init, LF_diff, LF_foot, 120, Lcolor );
PlotLeg( LM_body_init, LM_diff, LM_foot, 180, Lcolor );
PlotLeg( LR_body_init, LR_diff, LR_foot, 240, Lcolor );

end


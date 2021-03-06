function [ ] = PlotLeg( body_init, foot_delta, body_angle, color )

%foot_delta_t = TransformCoord(foot_delta, body_angle);

[ CoxaAngle,FemurAngle,TibiaAngle ] = LegIK( foot_delta);
%disp(['body_angle: ', num2str(body_angle),' Coxa: ', num2str(CoxaAngle), ' Femur: ', num2str(FemurAngle), ' Tibia: ', num2str(TibiaAngle)]);

[ FemurPt,TibiaPt,FootPt ] = LegFK( body_init, body_angle, CoxaAngle, FemurAngle, TibiaAngle );

LR_pts = [body_init(1,1), FemurPt(1,1), TibiaPt(1,1),  FootPt(1,1); 
          body_init(2,1), FemurPt(2,1), TibiaPt(2,1), FootPt(2,1); 
          body_init(3,1), FemurPt(3,1), TibiaPt(3,1), FootPt(3,1)];
plot3(LR_pts(1,:), LR_pts(2,:), LR_pts(3,:),'marker','o','color',color);
plot3(FootPt(1,1), FootPt(2,1), FootPt(3,1),'marker','x','color','k');

end


function winning_region(initial_angle, radius)
alpha = initial_angle;
win_points = [];
for t = 0.01:0.001: 2*pi
   if t < pi
       rou = radius*t;
       x_t = radius*cos(t); y_t = radius*sin(t);
       x_win = sqrt(rou^2/(1+(x_t/y_t)^2))+x_t;
       y_win = (-x_t/y_t)*(x_win-x_t)+y_t;
       new = [x_win; y_win];
       win_points = [win_points,new];
   elseif t == pi 
       x_win = -radius; y_win = pi*radius; 
       new = [x_win; y_win];
       win_points = [win_points,new];
   else
       rou = radius*t;
       x_t = radius*cos(t); y_t = radius*sin(t);
       x_win = -sqrt(rou^2/(1+(x_t/y_t)^2))+x_t;
       y_win = (-x_t/y_t)*(x_win-x_t)+y_t;
       if y_win < 1e-5
           break
       end
       new = [x_win; y_win];
       win_points = [win_points,new]; 
   end
end
   win_points_neg = [win_points(1,:); -win_points(2,:)];
   rotation = [cos(alpha), -sin(alpha);
               sin(alpha), cos(alpha)];
   win_points = rotation * win_points;
   win_points_neg = rotation * win_points_neg;
   plot(win_points(1,:),win_points(2,:));
   plot(win_points_neg(1,:),win_points_neg(2,:), 'r');
   drawnow;
end
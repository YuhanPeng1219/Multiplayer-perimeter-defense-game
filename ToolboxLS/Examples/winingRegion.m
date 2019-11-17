[row, col] = find(abs(ans)<0.02);
figure;
for k = 0 : 0.01 : pi
    x = cos(k); y = sin(k);
    scatter(x,y,'o');
    hold on;
end

for i = 1:size(row)
    R = row(i)*3.5/51+1;
    theta = col(i)*pi/46;
    
   scatter(R*cos(theta), R*sin(theta)) 
   hold on
end
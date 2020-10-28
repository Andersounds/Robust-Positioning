Rx=@(x) [1,0,0; 0 cosd(x),-sind(x);0,sind(x),cosd(x)];
Ry=@(x) [cosd(x), 0, sind(x);0, 1, 0; -sind(x), 0, cosd(0)];
Rz=@(x) [cosd(x), -sind(x), 0; sind(x), cosd(x), 0;0, 0,1];


angle = 37;
a = angle;
R1 = Rx(a)*Ry(a)*Rz(a);     %Normal that is working
R2 = Rz(-a)*Ry(-a)*Rx(-a);  %Flip order and sign
%R3 = Rz

 %T_MAT*vx' % Rotated to UAV it should be straight in +y direction
close all

dirs = [18,23,27,28];
figure(5);
set(5,'Position',[1,1,850,400],'PaperUnits','centimeters','PaperSize',[29, 15]);
for i=1:length(dirs)

disp('Reading true path file...');
%Read file
%%%%  BASEPATH  %%%%
basePath = '../data/';
dir_str = '20-11-3-sim/';%20-04-09/';
nmbrs_str = ['20-04-09-',num2str(dirs(i)),'/'];
d_true_file_full = [basePath,dir_str,nmbrs_str,'AZIPE_log.csv'];
d_ref= csvread(d_true_file_full,1,0);
% data columns in file
tcolR = 1;
xcolR = 2;
ycolR = 3;
zcolR = 4;
rollcolR = 5;
pitchcolR = 6;
yawcolR = 7;
%Create data vectors
t_ref = d_ref(:,tcolR)./1000;
x_ref = d_ref(:,xcolR);
y_ref = d_ref(:,ycolR);
z_ref = d_ref(:,zcolR);
roll_ref = d_ref(:,rollcolR);
pitch_ref = d_ref(:,pitchcolR);
yaw_ref = d_ref(:,yawcolR);


figure(5)
subplot(2,2,i)

n = 7;
x_ref_spaced = x_ref(1:n:end);
y_ref_spaced = y_ref(1:n:end);
yaw_ref_spaced = yaw_ref(1:n:end);
quiver(x_ref_spaced,y_ref_spaced,cos(yaw_ref_spaced),sin(yaw_ref_spaced)); hold on
plot(x_ref,y_ref,'color','k'); grid on; hold on
%plot3(x_ref,y_ref,-z_ref,'color','k'); grid on;axis equal
%quiver3(x_ref,y_ref,-z_ref,cos(yaw_ref),sin(yaw_ref),zeros(size(yaw_ref)));
title(['Ground truth, dataset ',num2str(i)])
xlabel('x-direction [m]')
ylabel('y-direction [m]')
end




        
outpath = '../../../../../Documentation/Report_presentations/AndersonThesis/Texter/Implementation/Figures/';
%outpath = '../../../../../Documentation/Report_presentations/';
filetitle = ['groundtruthPaths'];
fulltitle = [outpath,filetitle,'.pdf'];        
        
   
%saveas(nmbr3,fulltitle)
print(5,'-dpdf','-bestfit',fulltitle)


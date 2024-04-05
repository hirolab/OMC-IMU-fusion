%% Extrinsic IMU-OMC calibration
clear, %clc
clc
close all
addpath(genpath("C:\Users\garamizo\Documents\GitHub\OMC_IMU_fusion"))

% load('C:\Users\garamizo\Documents\GitHub\cdprosthesis-desktop\MATLAB\data\cable_driven_prosthesis_calib_0510.mat'); i = 3;
% load('C:\Users\garamizo\Documents\GitHub\cdprosthesis-desktop\MATLAB\data\cable_driven_prosthesis_calib_0602.mat'); i = 1;
% load('C:\Users\garamizo\Documents\GitHub\cdprosthesis-desktop\MATLAB\data\cable_driven_prosthesis4.mat'); i = 1;
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\data\cable_driven_prosthesis_calib_0627_imu.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\data\calibs_0902\pros_calib2_0902.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\eskf\Aim2022_paper_files\calib_02_06\cable_driven_prosthesis_020622.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\030623calibrations\cable_driven_prosthesis_031823_enc_imu_sg_walk_calibs.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\032523calibs\data\cable_driven_prosthesis_enc_imu_walk_032523.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\030623calibrations\data\cable_driven_prosthesis_031923_imu_sg_walk_Calib.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\041223calibs\data\cable_driven_prosthesis_041223_sg_imu.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\042723\data\cable_driven_prosthesis_enc_imu_sg.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\043023\data\cable_driven_prosthesis_imu.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\111523\data\cable_driven_prosthesis_11523_imu.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\111523\data\cable_driven_prosthesis_112123.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\113023\data\cable_driven_prosthesis_113023.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\031124\data\cable_driven_prosthesis_imu_enc.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\State_Estimation\data\johnny\johnny_struct.mat')
% load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\040524\data\cable_driven_prosthesis_imu_enc_europa.mat')
load('C:\Users\hiro\Documents\cdprosthesis-desktop\MATLAB\calibration_files\040524\data\cable_driven_prosthesis_imu.mat')

i = 1;
t = trial(1);
figure,subplot(221)
plot(t.packet.t_sensor_sync,t.packet.W2)
subplot(222)
plot(t.mtime,t.squat)



subplot(223)
plot(t.packet.t_sensor_sync,t.packet.A1)
subplot(224)
plot(t.mtime,t.fquat)


%%
close all
clc
% squat = fillmissing(t.squat,'spline');
% strans = fillmissing(t.strans,'spline');
% mstrans = fillmissing(t.mstrans,'spline');

cal = Calibration_OMC_IMU(t.mtime, t.squat, t.strans,t.mstrans, t.packet.t_sensor_sync, ...
                                                                 t.packet.W2, ...
                                                                 t.packet.A2);

% cal = Calibration_OMC_IMU(t.mtime, t.squat, t.strans,t.mstrans,t.itime_t_sensor_sync, ...
%                                                                 t.w2, ...
%                                                                  t.a2);
                                                             
%           cal.trange_ = [25, 150];
cal.trange_ = [10, 90];
tic
[cq1, cs1, cwbias1, cabias1, Tw1, Ta1, r1, g131, tshift1, ri1, x] = cal.calibrate_LM();
[Tw1, r1]    
toc

imucalib = struct ('cq1',cq1,'cs1',cs1,'cwbias1',cwbias1,'cabias1',cabias1,'Tw1',Tw1, 'Ta1',Ta1,'r1',r1,'g131',g131,'tshift1',tshift1,'ri1',ri1,'x',x);

uisave('imucalib','scalib.mat')
clear cal 

%%
% fquat = fillmissing(t.fquat,'linear');
% ftrans = fillmissing(t.ftrans,'linear');
cal = Calibration_OMC_IMU(t.mtime, t.fquat,t.ftrans, t.mftrans, t.packet.t_sensor_sync, ...
                                                                 t.packet.W1, ...
                                                                 t.packet.A1);
% cal.trange_ = [15, 90];

% fquat = fillmissing(t.fquat,'linear');
% ftrans = fillmissing(t.ftrans,'linear');
clc
% cal = Calibration_OMC_IMU(t.mtime, t.fquat,t.ftrans, t.mftrans, t.t_sensor_sync, ...
%                                                                 t.w1, ...
%                                                                  t.a1);
 cal.trange_ = [10, 90];
                                                            
tic
[cq1, cs1, cwbias1, cabias1, Tw1, Ta1, r1, g131, tshift1, ri1, x] = cal.calibrate_LM();
[Tw1, r1]    
toc

imucalib = struct ('cq1',cq1,'cs1',cs1,'cwbias1',cwbias1,'cabias1',cabias1,'Tw1',Tw1, 'Ta1',Ta1,'r1',r1,'g131',g131,'tshift1',tshift1,'ri1',ri1,'x',x);

uisave('imucalib','fcalib.mat')
toffset = tshift1*cal.dT_;

return



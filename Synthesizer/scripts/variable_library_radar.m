%% radar Field of View (FoV)
radar_FoV_deg = [58,121;76,107]; % azimuth and elevation field of view
radar_FoV_rad = radar_FoV_deg/180*pi; % angular FoV in radian
radar_FoV_rho = [3,15-0.125]; % range field of view

N_phi = 64; % number of azimuth bins in the output radar heatmap
N_theta = 32; % number of elevation bins in the output radar heatmap

phi_deg = linspace(radar_FoV_deg(1,1),radar_FoV_deg(1,2),N_phi); % azimuth axis of the output radar heatmap in degree
theta_deg = linspace(radar_FoV_deg(2,1),radar_FoV_deg(2,2),N_theta); % elevation axis of the output radar heatmap in degree
phi = phi_deg/180*pi;
theta = theta_deg/180*pi;

phi_res_deg = 1; % azimuth resolution = 1 degree
theta_res_deg = 1; % elevation resolution = 1 degree

%% FMCW radar parameters
c = 3e8; % speed of light
fc = 60*1e9; % radar center frequeny
lambda = c/fc; % radar wavelength

BW = 1.2e9; % FMCW bandwidth = 1.2 GHz
rho_res = c/2/BW; % range resolution
Ts = 0.8e-3; % FMCW sweep time = 0.8ms
As = BW/Ts; % FMCW frequency sweep slope
Rs = 5e5; % Baseband sampling rate = 500kHz

N_FFT = Ts*Rs; % FFT length / number of range bins
range_bin = linspace(0,(Rs-Rs/N_FFT),N_FFT) /As *c /2; % range axis
range_bin_FoV = find((range_bin >= radar_FoV_rho(1))&(range_bin <= radar_FoV_rho(2))); % select range bins in the range field of view
rho =  range_bin(range_bin_FoV); % range axis of output radar heatmap
N_rho = length(rho); % number of range bins in the output radar heatmap

%% antenna array
TX_pos = [0.44,0,0]; % TX antenna position. 
% For our custombuilt radar, TX is placed 44cm to the right of the origin (RX array center)

ary_siz = [40,40]; % emulated antenna array size
ary_stp = [2.5e-3,2.5e-3]; % emulated antenna array spacing

ary_x_idx = (1:ary_siz(1))-ceil(ary_siz(1)/2);
ary_x_idx = ary_x_idx.';

ary_z_idx = (1:ary_siz(2))-ceil(ary_siz(2)/2);



  
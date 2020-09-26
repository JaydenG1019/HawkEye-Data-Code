function radar_heatmap = radar_dsp(signal_array)
% Radar signal processing 
% Input: "signal_array" radar signal from the antenna array
% Output: "radar_heatmap" 3D radar heatmap in the spherical coordinates [rho, phi, theta]

    variable_library_radar; % load radar configurations
    
    range_FFT_array = fft(signal_array,N_FFT,1); % range FFT
    range_FFT_array = range_FFT_array(range_bin_FoV,:,:); % select range bins in the field of view

    %% Angle of Arrival Estimation
    radar_heatmap = zeros(N_rho,N_phi*N_theta);

    x_idx = reshape(array_x_idx,1,[]);
    z_idx = reshape(array_z_idx,1,1,[]);

    for kp = 1:N_phi
        for kt = 1:N_theta
            Vec = exp(1j*(2*pi*(array_gap(1)*x_idx* cos(phi(kp)) * sin(theta(kt)) + array_gap(2)*z_idx*cos(theta(kt)))/lambda));
            VecR = repmat(Vec,N_rho,1);
            radar_heatmap(:,kp,kt) = sum(range_FFT_array.*VecR,[2,3]);
        end
    end
end
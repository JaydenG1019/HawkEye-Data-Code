function signal_array = simulate_radar_signal(reflector_cart_v)
% Synth Radar signal

% Simualte received radar signal in the receiver antenna array
% Input: "reflector_cart_v" coordinates of point reflectors in the scene 
% Output: "signal_array" FMCW beat signals in the antenna array

    variable_library_radar; % load radar configurations
        
    tf = 0:1/Rs:(Ts-1/Rs); tf = tf.'; % time axis
    t_matrix = repmat(tf,1,array_size(1),array_size(2));

    array_x_m = repmat(array_x_idx*array_gap(1),1,array_size(2));
    array_z_m = repmat(array_z_idx*array_gap(2),array_size(1),1);

    signal_array = zeros(length(tf),array_size(1),array_size(2));

    for kp = 1:length(reflector_cart_v)
        d_TX2reflector = sqrt((TX_pos(1)-reflector_cart_v(kp,1)).^2+(TX_pos(2)-reflector_cart_v(kp,2)).^2+ (TX_pos(3)-reflector_cart_v(kp,3)).^2);
        % distance between the TX antenna and the point reflector
        d_RX2reflector = reshape(sqrt((array_x_m-reflector_cart_v(kp,1)).^2 + reflector_cart_v(kp,2)^2 + (array_z_m-reflector_cart_v(kp,3)).^2),[1,array_size(1),array_size(2)]);
        % distance between each RX antenna and the point reflector
        tau = repmat((d_TX2reflector+d_RX2reflector)/c,[length(tf),1,1]); % round trip Time of Flight (ToF)
        path_loss = repmat(1/d_TX2reflector/d_RX2reflector,[length(tf),1,1]);
        pt_signal = (path_loss.*exp(1j * 2 * pi * fc * tau)).* exp(1j * 2*pi * As * t_matrix .* tau); % beat signal from a single point reflector
        signal_array = signal_array + pt_signal; % summing up signals from all point reflector to get the received signal in the antenna array
    end
    
    signal_array = awgn(signal_array,-10); % introduce noise
end
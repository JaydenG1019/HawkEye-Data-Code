N_CAD_car=38; % number of CAD models of cars 
N_placement_car = 10; % # of placement we create with every selected car/group of cars

% angle of rotation for every model
rotate_ang_coarse = [0:45:315];
rotate_ang_fine = [-5:5];
rotate_ang = [];
for k_rotate_ang_fine = rotate_ang_fine
    rotate_ang = [rotate_ang,rotate_ang_coarse+k_rotate_ang_fine];
end
rotate_ang = sort(rotate_ang);

% inline function for 2D rotation
rotate2d =  @(x, M) (x(:, 1:2) * M);

% structure of point cloud and information of the car CAD model
car_v_struct = struct('cart_v',[], ... % cartesian coordinates vector
                'N_pt',0, ... % # of points in the model
                'bbox',[], ... % bounding box of the car 
                'lim',[], ... % min and max xyz coordinates
                'CAD_idx',0, ...
                'rotate',[], ... % degree rotated
                'translate',[]); % distance translated

variable_library_radar;

signal_path = '../results/signal/test'; % path to save the built scenes
heatmap_path = '../results/heatmap/test'; % path to save the built scenes
GT_path = '../results/ground_truth/test'; % path to save the built scenes


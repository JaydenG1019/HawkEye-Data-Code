function [sph_pwr, visible_sph_v] = main
    % Copyright (c) 2018-2020 Junfeng Guan, Sohrab Madani, Suraj Jog, Saurabh Gupta, 
    % Haitham Hassanieh, University of Illinois at Urbana-Champaign
    % 
    % Permission is hereby granted, free of charge, to any person obtaining a copy
    % of this software and associated documentation files (the "Software"), to deal
    % in the Software without restriction, including without limitation the rights
    % to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    % copies of the Software, and to permit persons to whom the Software is
    % furnished to do so, subject to the following conditions:
    % 
    % The above copyright notice and this permission notice shall be included in
    % all copies or substantial portions of the Software.
    % 
    % THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    % IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    % FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    % AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    % LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    % OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    % THE SOFTWARE.

    close all; clear; clc;
    addpath('functions');

    variable_library;

    for CAD_idx = 1:N_CAD_car
        
        % load the surface model
        load(sprintf('../CAD/CAD_model_%d.mat',CAD_idx));
        
        % CAD models are loaded as point clouds of size N_pt by 3, where N_pt
        % is the number of points and 3 values are the cartesian coordinates
        % unit is mm

        % store point cloud in pc (point cloud) structure
        car_v = car_v_struct;
        car_v.CAD_idx = CAD_idx;
        car_v.N_pt = length(cart_v);
        car_v.cart_v = cart_v;
        car_v.lim = [min(cart_v);max(cart_v)]; % find the limits in all three dimensions 
        [bbox_x, bbox_y, bbox_z] = meshgrid(car_v.lim(:,1),car_v.lim(:,2),car_v.lim(:,3)); % 8 vertices of the bounding box of the point cloud
        car_v.bbox = [bbox_x(:), bbox_y(:), bbox_z(:)]; 
        clear cart_v bbox N_pt car_idx;
        car1_v_origin = car_v;

        for ks = 1:N_placement_car
            car_scene_v = car1_v_origin;

            %% Rotate        
            car_scene_v.rotate = rotate_ang(randi(length(rotate_ang))); % randomly select a rotation angle and store it in the pc structure
            rotate_m_c1 = rotation_matrix(car_scene_v.rotate/180*pi); % create rotation matrix
            car_scene_v.cart_v(:,1:2) = rotate2d(car_scene_v.cart_v, rotate_m_c1); % rotate the point cloud 
            car_scene_v.bbox(:,1:2) = rotate2d(car_scene_v.bbox, rotate_m_c1); % rotate the bounding box
            car_scene_v.lim = [min(car_scene_v.cart_v);max(car_scene_v.cart_v)]; % update the limits in all three dimensions

            %% Translation
            trans_x_rng = (-4000 - car_scene_v.lim(1,1)):500:(4000 - car_scene_v.lim(2,1)); % range of translation along x axis
            trans_y_rng = (3500 - car_scene_v.lim(1,2)):500:(12000 - car_scene_v.lim(2,2)); % range of translation along y axis
            trans_x = trans_x_rng(randi(length(trans_x_rng))); % randomly select a translation distance along x axis
            trans_y = trans_y_rng(randi(length(trans_y_rng))); % randomly select a translation distance along y axis
            trans_z = -1250; % translate the point cloud -1250mm to compensate for the height of our radar 

            % translate
            car_scene_v.translate = [trans_x, trans_y, trans_z]; % store translation information in the pc structure
            car_scene_v.cart_v = car_scene_v.cart_v + car_scene_v.translate; % translate the point cloud
            car_scene_v.bbox = car_scene_v.bbox + car_scene_v.translate; % translate the bounding box
            car_scene_v.lim = [min(car_scene_v.cart_v);max(car_scene_v.cart_v)]; % update the limits in all three dimensions
            
            % convert unit from mm to m
            car_scene_v.cart_v = car_scene_v.cart_v/1000; 
            car_scene_v.bbox = car_scene_v.bbox/1000; 

%             % Visulize the rotated and translated point cloud
%             figure; 
%             cart_v_plot = datasample(car_scene_v.cart_v, 1000); % downsampling when plotting
%             scatter3(cart_v_plot(:,1),cart_v_plot(:,2),cart_v_plot(:,3)); hold on;
%             scatter3(car_scene_v.bbox(:,1), car_scene_v.bbox(:,2),car_scene_v.bbox(:,3)); hold on;
%             axis equal;
            
            %% Modle radar point reflectors in the scene
            [visible_cart_v] = remove_occlusion(car_scene_v); % remove occluded body of the car
            try
                reflector_cart_v = model_point_reflector(visible_cart_v,car_scene_v.bbox); % model point reflectors that reflect back to the radar receiver
            catch
                continue;
            end

            if isempty(reflector_cart_v)
                continue;
            end

            %% Simualte received radar signal in the receiver antenna array            
            signal_array = simulate_radar_signal(reflector_cart_v);
            
            %% Radar signal processing, generating 3D radar heatmaps
            sph_pwr = radar_dsp(signal_array);
        end
    end
end

function [visible_cart_v] = remove_occlusion(car_scene_v)
% Remove occluded body of the car
% Input: 'car_scene_v' point cloud of the car in the scene
% Output: 'visible_cart_v' point cloud of the visible body car to the radar (catesian coordinates)
    
    % convert cartesian coordinates of the point cloud to spherical coordinates
    car_scene_v.sph_v = zeros(car_scene_v.N_pt,3); 
    [car_scene_v.sph_v(:,1),car_scene_v.sph_v(:,2),car_scene_v.sph_v(:,3)] = cart2sph(car_scene_v.cart_v(:,1),car_scene_v.cart_v(:,2),car_scene_v.cart_v(:,3));
    car_scene_v.sphlim = [min(car_scene_v.sph_v);max(car_scene_v.sph_v)]; % limits in all three dimensions in the spherical coordinates            
  
    % convert the point cloud into a grid of 0s and 1s 
    
    % define voxel size of the grid
    phi_res = 0.2/180*pi;
    theta_res = 0.5/180*pi;
    rho_res = 0.02;
    
    sph_m_phi = car_scene_v.sphlim(1,1):phi_res:car_scene_v.sphlim(2,1)+phi_res;
    sph_m_theta = car_scene_v.sphlim(1,2):theta_res:car_scene_v.sphlim(2,2)+theta_res;
    sph_m_rho = car_scene_v.sphlim(1,3):rho_res:car_scene_v.sphlim(2,3)+rho_res;
    sph_m_size = [length(sph_m_phi),length(sph_m_theta),length(sph_m_rho)];
    sph_m = zeros(sph_m_size);

    phi_m_idx = round((car_scene_v.sph_v(:,1) - car_scene_v.sphlim(1,1))/phi_res)+1;
    theta_m_idx = round((car_scene_v.sph_v(:,2) - car_scene_v.sphlim(1,2))/theta_res)+1;
    rho_m_idx = round((car_scene_v.sph_v(:,3) - car_scene_v.sphlim(1,3))/rho_res)+1;
    for k_pt = 1:car_scene_v.N_pt
        sph_m(phi_m_idx(k_pt),theta_m_idx(k_pt),rho_m_idx(k_pt)) = 1;
    end

    %% Find first voxel with car body in every angle
    visible_sph_m = zeros(size(sph_m));
    for kphi = 1:sph_m_size(1)
        for ktheta = 1:sph_m_size(2)
            krho = find(sph_m(kphi,ktheta,:)>0,1);
            visible_sph_m(kphi,ktheta,krho) = sph_m(kphi,ktheta,krho);
        end
    end

    visible_sph_m_idx = find(visible_sph_m);
    sph_v_idx = [];
    [sph_v_idx(:,1),sph_v_idx(:,2),sph_v_idx(:,3)] = ind2sub(sph_m_size,visible_sph_m_idx);
    visible_sph_v = [sph_m_phi(sph_v_idx(:,1));sph_m_theta(sph_v_idx(:,2));sph_m_rho(sph_v_idx(:,3))].';

    visible_cart_v = zeros(size(visible_sph_v));
    [visible_cart_v(:,1),visible_cart_v(:,2),visible_cart_v(:,3)] = sph2cart(visible_sph_v(:,1),visible_sph_v(:,2),visible_sph_v(:,3));

%     figure;
%     scatter3(visible_cart_v(:,1),visible_cart_v(:,2),visible_cart_v(:,3)); hold on
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');

end
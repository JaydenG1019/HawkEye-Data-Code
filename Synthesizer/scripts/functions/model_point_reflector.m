function blb_cart_v = model_point_reflector(visible_cart_v,bbox)
% Model radar point reflectors in the scene
% Input: 'visible_cart_v': point cloud of the visible body of the car (cartesian coordinates) 
%        'bbox': bounding box of the car
% Output: 'blb_cart_v': blobs of radar point reflectos

    % local function find the distance from a point pt to a line determined by 2 points v1 and v2
    function d = point_to_line(pt, v1, v2)
          a = [v1 - v2,0];
          b = [pt - v2,0];
          d = norm(cross(a,b)) / norm(a);
    end
    
    %% Blobs center
    blb_ctr_cart = []; % 
    N_pt_car = length(visible_cart_v);
    pt_specular = zeros(N_pt_car,1);

%     box_close = sqrt((bbox(2:4,1)-bbox(1,1)).^2+(bbox(2:4,2)-bbox(1,2)).^2);
%     [~,box_close] = sort(box_close);
%     bbox(2:4,1:2) = bbox(box_close+1,1:2);

    bbox_edge = zeros(4,1); % 4 edges of the bounding box represented as a complex number
    bbox_edge_cart = zeros(4,2,2); % coordinates of the vertices of the bounding box edges 

    bbox_edge_cart(1,:,:) = squeeze(bbox([1,2],1:2));
    bbox_edge_cart(2,:,:) = squeeze(bbox([1,3],1:2));
    bbox_edge_cart(3,:,:) = squeeze(bbox([2,4],1:2));
    bbox_edge_cart(4,:,:) = squeeze(bbox([3,4],1:2));
    for ke = 1:4
        bbox_edge(ke,:) = bbox_edge_cart(ke,2,1)-bbox_edge_cart(ke,1,1)+1j*(bbox_edge_cart(ke,2,2)-bbox_edge_cart(ke,1,2));
    end

    % find the specularity for each point
    for kp = 1:N_pt_car

        if min(sqrt((visible_cart_v(kp,1)-bbox(1:4,1)).^2+(visible_cart_v(kp,2)-bbox(1:4,2)).^2))<0.5
            % if the point is very close to a corner of the bounding box
            pt_specular(kp) = 0;
        else
            % find the distance between the point and all 4 edges to
            % find the nearest edge
            pt_edge_dist = zeros(4,1);
            for ke = 1:4
                pt_edge_dist(ke) = point_to_line([visible_cart_v(kp,1),visible_cart_v(kp,2)], squeeze(bbox_edge_cart(ke,1,:)).', squeeze(bbox_edge_cart(ke,2,:)).');
            end
            [~,edge_nearest] = min(pt_edge_dist);
            % find the angle between the insertion angle and angle of the nearest edge  
            pt_specular(kp) = abs(angle(bbox_edge(edge_nearest)) - angle(visible_cart_v(kp,1)+1j*visible_cart_v(kp,2)));
            pt_specular(kp) = abs(mod(pt_specular(kp),pi)/pi*180-90);
            % find the elevation angle theta
            pt_theta = angle(sqrt(visible_cart_v(kp,1)^2+visible_cart_v(kp,2)^2)+1j*abs(visible_cart_v(kp,3)));
            pt_theta = abs(mod(pt_theta,pi)/pi*180);
            % the larger one is the specularity of the point
            pt_specular(kp) = max(pt_specular(kp),pt_theta);
        end
    end

    % Blob center
    % Find the centers of the blobs
    blb_ctr = find(pt_specular==0); % all corners are selected
    blb_ctr_cart = [];
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(10,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;visible_cart_v(blb_ctr_idx,:)];
    end

    blb_ctr = find((pt_specular>0)&(pt_specular<15));
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(20,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;visible_cart_v(blb_ctr_idx,:)];
    end

    blb_ctr = find((pt_specular>15)&(pt_specular<25));
    if ~isempty(blb_ctr)
        blb_ctr_idx = datasample(blb_ctr,min(5,length(blb_ctr)));
        blb_ctr_cart = [blb_ctr_cart;visible_cart_v(blb_ctr_idx,:)];
    end

    %%
    if ~isempty(blb_ctr_cart)

%         figure;
%         scatter3(blb_ctr_cart(:,1),blb_ctr_cart(:,2),blb_ctr_cart(:,3)); hold on;
%         title('Center of blobs');

        blb_size = 0.3; % blob size around the center
        blb_cart_v = [];
        for kb = 1:size(blb_ctr_cart,1)
            dis_pt2blb = (visible_cart_v(:,1) - blb_ctr_cart(kb,1)).^2 + (visible_cart_v(:,2) - blb_ctr_cart(kb,2)).^2 + (visible_cart_v(:,3) - blb_ctr_cart(kb,3)).^2;
            ptInBlb = find(dis_pt2blb < blb_size^3);
            blb_cart_v = [blb_cart_v;[visible_cart_v(ptInBlb,1),visible_cart_v(ptInBlb,2),visible_cart_v(ptInBlb,3)]];
        end
        blb_cart_v = unique(blb_cart_v,'row');
    end
end
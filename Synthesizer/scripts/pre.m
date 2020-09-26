for CAD_idx = 5:N_CAD_car
        clear cart cart_v;
        load(sprintf('../CAD/CAD_model_%d.mat',CAD_idx));
        dims = [dims(1),dims(3),dims(2)]; % real-world dimensions of the car bounding box/cart

        cart_size = size(cart); % size of the grid
        pt_idx = find(cart); clear cart_grid; % find points in the cart grid
        N_pt = length(pt_idx); % # of points
        [cart_v(:,1),cart_v(:,2),cart_v(:,3)] = ind2sub(cart_size,pt_idx); % convert the matrix into coordinates vectors 

        % convert pt coordinates to real-world dimension (mm)
        [~,dims_sort] = sort(dims);
        [~,cart_sort] = sort(cart_size);
        for kax = 1:3
            cart_uVec = dims(dims_sort(cart_sort == kax))/cart_size(kax); % find unit vector on each axis
            cart_v(:,kax) = cart_v(:,kax)*cart_uVec;
        end

        cart_v_lim = round([min(cart_v);max(cart_v)]);
        if diff(cart_v_lim(:,1)) < diff(cart_v_lim(:,2))
            cart_v(:,1:2) = rotate2d(cart_v, rotation_matrix(pi/2));
        end

        % find x-y plane center
        cart_v_xctr = round((min(cart_v(:,1))+max(cart_v(:,1)))/2);
        cart_v_yctr = round((min(cart_v(:,2))+max(cart_v(:,2)))/2);
        cart_v_zctr = min(cart_v(:,3));
        cart_v = cart_v - [cart_v_xctr, cart_v_yctr, cart_v_zctr];

    %     figure; 
    %     scatter3(cart_v(:,1), cart_v(:,2), cart_v(:,3));

        save(sprintf('../CAD/CAD_model_%d.mat',CAD_idx),'cart_v');
    end
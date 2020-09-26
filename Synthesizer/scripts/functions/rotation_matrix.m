function M = rotation_matrix(rotate_angle)
% create the rotation matrix given the rotation angle
    M = [cos(rotate_angle), -sin(rotate_angle); sin(rotate_angle), cos(rotate_angle)];
end
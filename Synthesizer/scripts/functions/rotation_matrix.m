function M = rotation_matrix(theta)
    M = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
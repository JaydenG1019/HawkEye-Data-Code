function d = point_to_line(pt, v1, v2)
      a = [v1 - v2,0];
      b = [pt - v2,0];
      d = norm(cross(a,b)) / norm(a);
end
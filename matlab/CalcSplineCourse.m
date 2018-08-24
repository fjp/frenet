function [rx, ry, ryaw, rk, s, oSpline] = CalcSplineCourse(x, y, ds) % d=0.1 default
    oSpline = cSpline2D(x, y);
    s = 0:ds:oSpline.s(end);

    rx = [];
    ry = [];
    ryaw = [];
    rk = [];
    for i_s = s
        [ix, iy] = oSpline.calc_position(i_s);
        rx(end+1) = ix;
        ry(end+1) = iy;
        ryaw(end+1) = oSpline.calc_yaw(i_s);
        rk(end+1) = oSpline.calc_curvature(i_s);
    end
end
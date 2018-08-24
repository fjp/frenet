classdef cSpline2D
    %CSPLINE2D Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = public)
        s
        sx
        sy
        
        ds
    end
    
    methods(Access = public)
        function obj = cSpline2D(x, y)
            obj.s = obj.calc_s(x, y);
            obj.sx = cSpline(obj.s, x);
            obj.sy = cSpline(obj.s, y);
        end
        
        function s = calc_s(obj, x, y)
        %%%%%%
        % Calculate running length s
        %%%%%%
            dx = diff(x);
            dy = diff(y);
            obj.ds = zeros(length(dx), 1);
            for i = 1:length(dx)
                idx = dx(i);
                idy = dy(i);
                obj.ds(i) = sqrt(idx^2 + idy^2);
            end     
            s = [0, cumsum(obj.ds)'];
        end

        function [x, y] = calc_position(obj, s)
        %%%%%%
        % calc position
        %%%%%%
            x = obj.sx.calc(s);
            y = obj.sy.calc(s);

        end

        function k = calc_curvature(obj, s)
        %%%%%%
        % calc curvature
        %%%%%%
            dx = obj.sx.calcd(s);
            ddx = obj.sx.calcdd(s);
            dy = obj.sy.calcd(s);
            ddy = obj.sy.calcdd(s);
            k = (ddy * dx - ddx * dy) / (dx^2 + dy^2);
        end

        function yaw = calc_yaw(obj, s)
        %%%%%%%%%%
        % calc yaw
        %%%%%%%%%%
            dx = obj.sx.calcd(s);
            dy = obj.sy.calcd(s);
            yaw = atan2(dy, dx);
        end


    end
    
end


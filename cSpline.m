classdef cSpline
    %CSPLINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access = public)
        a
        b
        c
        d
        w
        
        x
        y
        
        nx
    end
    
    methods(Access = public)
        
        function obj = cSpline(x, y)
            obj.b = [];
            obj.c = [];
            obj.d = [];
            obj.w = [];

            obj.x = x;
            obj.y = y;

            obj.nx = length(x);  % dimension of x
            h = diff(x);

            % calc coefficient c
            obj.a = y;
            
            % calc coefficient c
            A = obj.calc_A(h);
            B = obj.calc_B(h);
            obj.c = A\B;
            obj.c = obj.c';
            %  print(obj.c1)

            % calc spline coefficient b and d
            for i = 1:(obj.nx-1)
                obj.d(end+1) = (obj.c(i+1) - obj.c(i)) / (3.0 * h(i));
                tb = (obj.a(i+1) - obj.a(i)) / h(i) - h(i) * ...
                    (obj.c(i+1) + 2.0 * obj.c(i)) / 3.0;
                obj.b(end+1) = tb;
            end
            
%             disp(['a ', num2str(obj.a)]);
%             disp(['b ', num2str(obj.b)]);
%             disp(['c ', num2str(obj.c)]);
%             disp(['d ', num2str(obj.d)]);
        end
        
        
        function result = calc(obj, t)
        %%%%%%%%%%%%%
        % Calc position
        % if t is outside of the input x, return None
        %%%%%%%%%%%%%%%%%

            if (t < obj.x(1))
                result = NaN;
                return;
            elseif (t > obj.x(end))
                result = NaN;
                return;
            end

            i = obj.search_index(t);
            dx = t - obj.x(i);
            result = obj.a(i) + obj.b(i)*dx + obj.c(i)*dx^2.0 + obj.d(i)*dx^3.0;
        end

        function result = calcd(obj, t)
        %%%%%%%%%%
        % Calc first derivative
        % if t is outside of the input x, return None
        %%%%%%%%%%%%

            if (t < obj.x(1))
                result = NaN;
                return;
            elseif (t > obj.x(end))
                result = NaN;
                return;
            end

            i = obj.search_index(t);
            dx = t - obj.x(i);
            result = obj.b(i) + 2.0*obj.c(i)*dx + 3.0*obj.d(i)*dx^2.0;
        end

        function result = calcdd(obj, t)
        %%%%%%%%%%
        % Calc second derivative
        %%%%%%%%%%

            if (t < obj.x(1))
                result = NaN;
                return;
            elseif (t > obj.x(end))
                result = NaN;
                return;
            end

            i = obj.search_index(t);
            dx = t - obj.x(i);
            result = 2.0*obj.c(i) + 6.0*obj.d(i)*dx;
        end

        function idx = search_index(obj, x)
        %%%%%%%%
        % search data segment index
        %%%%%%%%
            % return bisect.bisect(obj.x, x) - 1
            [~, idx] = min(abs(obj.x - floor(x-2.9)));
            if (idx == obj.nx)
                idx = idx - 1;
            end
            %disp(idx)
        end
        

        function A = calc_A(obj, h)
        %%%%%%%
        % calc matrix A for spline coefficient c
        %%%%%%%
        
            A = zeros(obj.nx, obj.nx);
            A(1,1) = 1.0;
            for i = 1:(obj.nx - 1)
                if i ~= (obj.nx - 1)
                    A(i + 1, i + 1) = 2.0 * (h(i) + h(i + 1));
                end
                A(i + 1, i) = h(i);
                A(i, i + 1) = h(i);
            end

            A(1, 2) = 0.0;
            A(obj.nx, obj.nx - 1) = 0.0;
            A(obj.nx, obj.nx) = 1.0;
            
            %A
        end
                

        function B = calc_B(obj, h)
        %%%%%%%
        % calc matrix B for spline coefficient c
        %%%%%%%
            B = zeros(obj.nx, 1);
            for i = 1:(obj.nx - 2)
                B(i+1) = 3.0 * (obj.a(i+2) - obj.a(i+1)) / ...
                    h(i+1) - 3.0 * (obj.a(i+1) - obj.a(i)) / h(i);
            end
            
            %B
        end
        
    end
    
end


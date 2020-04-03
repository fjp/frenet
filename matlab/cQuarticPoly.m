classdef cQuarticPoly
    %CQUARTICPOLY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % coefficients (c0, c1, c2, c3, c4)
        c
    end
    
    methods (Access = public)
        
        function obj = cQuarticPoly(xi0, dxi0, ddxi0, dxiT, ddxiT, T) 
            c012 = [xi0; dxi0; ddxi0 / 2.0];
            
            M1 = [0, 1, 2*T; 
                  0, 0, 2  ];
  
            M2 = [3*T^2,  4*T^3;
                    6*T, 12*T^2];

            c34 = M2\([dxiT; ddxiT] - M1 * c012);
            obj.c = [c012; c34];
        end
        
        function x = X(obj, t)
            x = obj.c(1) + obj.c(2).*t + obj.c(3).*t.^2 + obj.c(4).*t.^3 + obj.c(5).*t.^4;   
        end
        
        function x = dX(obj, t)
            x = obj.c(2) + 2*obj.c(3).*t + 3*obj.c(4).*t.^2 + 4*obj.c(5).*t.^3;   
        end
        
        function x = ddX(obj, t)
            x = 2*obj.c(3) + 6*obj.c(4).*t + 12*obj.c(5).*t.^2;   
        end
        
        function x = dddX(obj, t)
            x = 6*obj.c(4) + 24*obj.c(5).*t;   
        end
        
    end
    
end
% Copyright (c) IDLab - University of Antwerp / imec. All rights reserved.
% Licensed under the MIT License.
% Author: Siemen Herremans

classdef SimplePID
    properties
        Kp
        Ki
        Kd
        prevError
        integral
    end
    
    methods
        function obj = SimplePID(Kp, Ki, Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.prevError = 0;
            obj.integral = 0;
        end
        
        function output = compute(obj, setpoint, measurement)
            % Calculate error
            error = setpoint - measurement;
            
            % Proportional term
            P = obj.Kp * error;
            
            % Integral term
            obj.integral = obj.integral + error;
            I = obj.Ki * obj.integral;
            
            % Derivative term
            D = obj.Kd * (error - obj.prevError);
            
            % PID output
            output = P + I + D;
            
            % Update previous error
            obj.prevError = error;
        end
    end
end
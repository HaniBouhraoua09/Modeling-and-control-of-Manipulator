%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
       function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame (Base to Goal)
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            % 1. Get the current Tool pose from the Geometric Model
            bTt = self.gm.getToolTransformWrtBase();
            
            % Extract Position vectors
            p_tool = bTt(1:3, 4);
            p_goal = bTg(1:3, 4);
            
            % Extract Rotation matrices
            % R_tool is bRt
            % R_goal is bRg
            R_tool = bTt(1:3, 1:3);
            R_goal = bTg(1:3, 1:3);
            
            %% Linear Error Computation
            % e_p = p_goal - p_tool (Calculated in Base Frame)
            e_p = p_goal - p_tool;
            
            %% Angular Error Computation (Axis-Angle with Frame Cancellation)
            
            % CHANGE: Perform "Cancellation" to find error in Tool Frame
            % tRg = tRb * bRg  -> (bRt)' * bRg
            % This follows the standard chain: tRg = inv(bRt) * bRg
            R_err = R_tool' * R_goal; 
            
            % Calculate the angle theta from the trace of the rotation matrix
            % trace(R) = 1 + 2*cos(theta)
            tr = R_err(1,1) + R_err(2,2) + R_err(3,3);
            cos_theta = (tr - 1) / 2;
            
            % Clamp for numerical stability
            if cos_theta > 1
                cos_theta = 1;
            elseif cos_theta < -1
                cos_theta = -1;
            end
            
            theta = acos(cos_theta);
            
            % Calculate the axis 'n' in the Tool Frame
            if theta < 1e-5
                % If angle is very small, orientation error is zero
                e_o_tool = [0; 0; 0];
            else
                % Extract axis 'n' from the skew-symmetric part of R_err
                % S_n = (R - R') / (2 * sin(theta))
                S_n = (R_err - R_err') / (2 * sin(theta));
                
                % Extract the vector n = [nx; ny; nz]
                nx = S_n(3, 2);
                ny = S_n(1, 3);
                nz = S_n(2, 1);
                n_tool = [nx; ny; nz];
                
                % Error vector in the Tool Frame
                e_o_tool = theta * n_tool;
            end
            
            % CHANGE: Project the tool-frame error back into the Base Frame
            % Since J is in Base Frame, we need: b_eo = bRt * t_eo
            e_o = R_tool * e_o_tool;
            
            %% Compute Reference Velocities
            % v_ref = K_l * error_linear
            v_ref = self.k_l * e_p;
            
            % w_ref = K_a * error_angular
            w_ref = self.k_a * e_o;
            
            % Combine into 6x1 vector
            x_dot = [v_ref; w_ref];
        end
    end
end
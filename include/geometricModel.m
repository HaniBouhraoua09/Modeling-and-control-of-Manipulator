%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt % Transformation from End-Effector to Tool
    end

    methods
        %% Constructor to initialize the geomModel property
        % Modified to accept eTt as per main.m structure
        function self = geometricModel(iTj_0, jointType, eTt)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                if nargin > 2
                    self.eTt = eTt;
                else
                    self.eTt = eye(4); % Default identity if not provided
                end
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        
        
        %% updateDirectGeometry
        function updateDirectGeometry(self, q)
            % GetDirectGeometryFunction
            % This method updates the matrices iTj.
            % Inputs:
            % q : joints current position ;
            % Save the current configuration
            self.q = q;

            % Loop through each joint to compute the local transformation
            for k = 1:self.jointNumber
                % Get the static transformation defined in BuildTree()
                T_static = self.iTj_0(:,:,k);
                
                q_k = q(k);

                % Calculate the variable transformation matrix (T_var)
                % check if the joint is Rotational (0) or Prismatic (1)
                if self.jointType(k) == 0
                    % Revolute Joint
                    % Rotation around the Z-axis by angle q_k
                    T_var = [cos(q_k) -sin(q_k)  0   0;
                             sin(q_k)  cos(q_k)  0   0;
                                0         0      1   0;
                                0         0      0   1];
                                
                elseif self.jointType(k) == 1
                    % Prismatic Joint 
                    % Translation along the Z-axis by distance q_k
                    T_var = [1   0   0   0;
                             0   1   0   0;
                             0   0   1   q_k;
                             0   0   0   1];
                else
                    error('Invalid joint type. Use 0 (Revolute) or 1 (Prismatic).');
                end
                
                %  Update the local transform : T_local = T_static * T_variable
                self.iTj(:,:,k) = T_static * T_var;
            end
        end
        
        %% getToolTransformWrtBase
        function [bTt] = getToolTransformWrtBase(self)
            % getToolTransformWrtBase function
            % outputs
            % bTt : transformation matrix from the manipulator base to the
            % tool
            
            % Get transformation from base to end-effector (flange)
            bTe = self.getTransformWrtBase(self.jointNumber);
            
            % Use the stored fixed transformation eTt
            % Chain rule: bTt = bTe * eTt
            bTt = bTe * self.eTt;
        end
        
        
        %% getTransformWrtBase
        function [bTk] = getTransformWrtBase(self,k)
            % GetTransformatioWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.
                if k == 0
                    bTk = eye(4);
                    return;
                end
            
                % Initialize base tranform as Identity
                bTk = eye(4);
            
                % Multiply all local tranfomrs matrices from 1  to link
                % number 
                for i = 1:k
                    bTk = bTk * self.iTj(:,:,i);
                end
        end
    end
end
%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end
    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        
        function bJi = getJacobianOfLinkWrtBase(self, i)
            %% getJacobianOfLinkWrtBase
            % Computes the Jacobian matrix bJi of link i wrt base.
       
            % Initialize Jacobian with correct size
            bJi = zeros(6, self.gm.jointNumber);
            
            % Get the position of the target link 'i'
            bTi = self.gm.getTransformWrtBase(i);
            p_e = bTi(1:3, 4);
            
            % Loop through all joints affecting this link ( j=1 to i )
            for j = 1:i
                % Get the transform of the previous Joint(Frame j-1) 
                if j == 1
                    bT_prev = eye(4); % Base frame for the first joint
                else
                    
                    bT_prev = self.gm.getTransformWrtBase(j);
                end
                
                % Extract z-axis and Position of the joint
                z_prev = bT_prev(1:3, 3); 
                p_prev = bT_prev(1:3, 4); 
                
                if self.gm.jointType(j) == 0
                    % Revolute Joint
                    % Linear v = z x (p_e - p_prev)
                    bJi(1:3, j) = cross(z_prev, (p_e - p_prev));
                    % Angular w = z
                    bJi(4:6, j) = z_prev;
                    
                elseif self.gm.jointType(j) == 1
                    % prismatic Joint
                    % Linear v = z
                    bJi(1:3, j) = z_prev;
                    % Angular w = 0
                    bJi(4:6, j) = zeros(3,1);
                end
            end
        end
        
        function updateJacobian(self)
            %% Update Jacobian function
            % Updates self.J to be the Jacobian of the end-effector (TOOL)
            
            % CRITICAL CHANGE FOR EXERCISE 2:
            % We cannot simply call getJacobianOfLinkWrtBase because that targets the Flange.
            % We must manually calculate the Jacobian targeting the TOOL TIP.
            
            % 1. Get the Tool Position (p_tool)
            bTt = self.gm.getToolTransformWrtBase();
            p_tool = bTt(1:3, 4); 
            
            % 2. Initialize Jacobian
            self.J = zeros(6, self.gm.jointNumber);
            
            % 3. Loop through all joints
            for j = 1:self.gm.jointNumber
                % Get the transform of the Joint(Frame j)
                if j == 1
                    bT_prev = eye(4);
                else
                    bT_prev = self.gm.getTransformWrtBase(j);
                end
                
                % Extract z-axis and Position of the joint axis
                z_prev = bT_prev(1:3, 3);
                p_prev = bT_prev(1:3, 4);
                
                if self.gm.jointType(j) == 0
                    % Revolute Joint
                    % Use p_tool instead of p_e (link position)
                    self.J(1:3, j) = cross(z_prev, (p_tool - p_prev));
                    self.J(4:6, j) = z_prev;
                    
                elseif self.gm.jointType(j) == 1
                    % Prismatic Joint
                    self.J(1:3, j) = z_prev;
                    self.J(4:6, j) = zeros(3,1);
                end
            end
        end
    end
end
%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); 
addpath('include/utils'); 

%% Compute the geometric model for the given manipulator
% Assumes BuildTree.m is implemented for the 7-DOF robot
iTj_0 = BuildTree();

disp('iTj_0')
disp(iTj_0);

% Joint 6 is Prismatic (1), others are Rotational (0)
jointType = [0 0 0 0 0 1 0]; 

% Initial Configuration
q = [pi/2, -pi/4, 0 , -pi/4, 0, 0.15, pi/4]';

% Control proportional gains
k_a = 0.4;
k_l = 0.4;

% Joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

%% Define the tool frame rigidly attached to the end-effector


eTt_tool = [ 0 -1  0  0  ;
             1  0  0  0  ; 
             0  0  1 0.13 ;
             0  0  0  1  ]; 


%% Define the goals

%  Goal definition (Phase 1: Tool) 
bOg_tool = [0.0087; 0.5808; 0.6714];
YPR_tool = [1.0363; 0.3218; -0.0077];
R_g_tool = YPRToRot(YPR_tool(1), YPR_tool(2), YPR_tool(3));
bTg_tool = [R_g_tool, bOg_tool; 0 0 0 1];

%  Goal definition (Phase 2: End-Effector) 
bOg_ee = [0.0202; 0.7779; 0.9927];
YPR_ee = [-0.3876; -0.3614; -0.3647];
R_g_ee = YPRToRot(YPR_ee(1), YPR_ee(2), YPR_ee(3));
bTg_ee = [R_g_ee, bOg_ee; 0 0 0 1];

%% Initialize Geometric Model (GM) and Kinematic Model (KM)
% Initialize geometric model with initial setup (Phase 1 default)
gm = geometricModel(iTj_0, jointType, eTt_tool);

% Initialize the kinematic model
km = kinematicModel(gm);

% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

%% Initialize control loop
% Simulation variables
samples = 100; % Increased samples for smoother simulation
t_start = 0.0;
t_end = 20.0;
dt = (t_end-t_start)/samples;
time = t_start:dt:t_end;

% Initializing plot
show_simulation = true; 
pm = plotManipulators(show_simulation); 
pm.initMotionPlot(time, bTg_tool(1:3,4), 'Phase 1');

% Define Tasks for the Loop (To avoid code duplication)
tasks(1).goal = bTg_tool;
tasks(1).tool_transform = eTt_tool; % Use defined tool offset
tasks(1).name = 'Phase 1 (Tool)';

tasks(2).goal = bTg_ee;
tasks(2).tool_transform = eye(4);   % Control EE directly -> Offset is Identity
tasks(2).name = 'Phase 2 (End-Effector)';

%% Kinematic Simulation Loop (Phased)

for phase = 1:length(tasks)
    
    disp(['Starting ', tasks(phase).name]);
    
    % 1. Update Goal and Tool for the current phase
    current_goal = tasks(phase).goal;
    
    % Update the tool transform in the Geometric Model
    % (This fulfills Note 1: handle change of tool control point)
    gm.eTt = tasks(phase).tool_transform; 
    
    % Reset convergence flag
    reached = false;

    for t = time
        % Updating geometric model 
        gm.updateDirectGeometry(q);

        % Get the cartesian error given an input goal frame
        % We use the current goal determined by the phase
        x_dot = cc.getCartesianReference(current_goal);

        % Update the jacobian matrix of the given model
        km.updateJacobian();

        % Inverse Kinematics: compute desired joint velocities 
        % Using Damped Least Squares or Pseudo-Inverse
        lambda = 0.01; 
        J_pinv = km.J' * inv(km.J * km.J' + lambda^2 * eye(6));
        q_dot = J_pinv * x_dot;

        % Simulating the robot (Integration)
        q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
        
        % Plot
        pm.plotIter(gm, km, t, q_dot); 

        % Check if goal is reached
        if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
            disp(['Reached Goal for ', tasks(phase).name]);
            disp(['Time: ', num2str(t)]);
            reached = true;
            break
        end
    end
    
    if ~reached
        disp(['Warning: Did not converge for ', tasks(phase).name]);
    end
    
    % Small pause between phases for visualization
    pause(1.0);
end

%% Final plots
pm.plotFinalConfig(gm, 'Final Configuration', 'Final');

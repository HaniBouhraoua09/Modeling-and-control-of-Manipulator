%% Plot Manipulators Class - GRAAL Lab
%% DO NOT MODIFY!!
classdef plotManipulators < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        cmap
        color
        cindex
        csize
        show_simulation
        bri
        x_dot_hist
        t_hist
    end

    methods
        function self = plotManipulators(show_simulation)
            self.show_simulation = show_simulation;
            self.x_dot_hist = [];
            self.t_hist = [];
        end

        % Constructor to initialize the geomModel property
        function initMotionPlot(self, t, bPoint, description)
            figure
            grid on 
            hold on
%             title('MOTION OF THE MANIPULATOR ' + description)
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis equal
            az = 48;
            el = 25;
            view(az,el)
            self.cindex = 1;
            self.csize = length(t);
            self.cmap = colormap(parula(self.csize));
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            plot3(bPoint(1), bPoint(2), bPoint(3), 'ro', ...
              'MarkerSize', 10, ...
              'MarkerFaceColor', 'red')

        end

        function plotIter(self, gm, km, i, q_dot)
            for j=1:gm.jointNumber
                bTi(:,:,j) = gm.getTransformWrtBase(j); 
            end
        
            bri(:,1) = [0; 0; 0];
            % Plot joints
            for j = 1:gm.jointNumber
                bri(:,j+1) = bTi(1:3,4,j);              
            end
            bTt = gm.getToolTransformWrtBase();
            bri(:,gm.jointNumber+2) = bTt(1:3,4); 

            if (rem(i,0.1) ~= 0)
                return;
            end
    
            % Plot links
            for j = 1:gm.jointNumber+1
                plot3(bri(1,j), bri(2,j), bri(3,j),'bo')           
            end
            plot3(bri(1,gm.jointNumber+2),bri(2,gm.jointNumber+2),bri(3,gm.jointNumber+2),'go') 
        
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            self.cindex = self.cindex + 1;
        
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', self.color)
            if self.show_simulation == true
                drawnow
            end
            self.bri = bri;

            x_dot_actual = km.J*q_dot;

            self.x_dot_hist = [self.x_dot_hist; (x_dot_actual(1:3)/norm(x_dot_actual(1:3)))',(x_dot_actual(4:6)/norm(x_dot_actual(4:6)))'];
            self.t_hist = [self.t_hist; i];

        end

        function plotFinalConfig(self, gm, description, agent)
            figure
            grid on 
            hold on
%             title('FINAL CONFIGURATION ' + description)
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis equal
            az = 48;
            el = 25;
            view(az,el)
            self.cindex = 1;
            for j = 1:gm.jointNumber+2
                plot3(self.bri(1,j), self.bri(2,j), self.bri(3,j),'bo')
                
            end
            
            self.color = self.cmap(mod(self.cindex,self.csize)+1,:);
            self.cindex = self.cindex + 1;
            
            bri = self.bri;
            line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', self.color)
           
            figW = 900;
            figH = 600;
            screenSize = get(0, 'ScreenSize');   % [left bottom width height]
            figX = (screenSize(3) - figW) / 2;
            figY = (screenSize(4) - figH) / 2;
            figure('Position', [figX figY figW figH])
%             sgtitle('DIRECTION OF THE ' + agent + ' VELOCITIES ' + description)
            
            % --- Angular velocities ---
            subplot(2,1,1)
            plot(self.t_hist, self.x_dot_hist(:,1:3), 'LineWidth', 1.2)
            legend('omega x', 'omega y', 'omega z')
            ylabel('Angular velocity')
            grid on
            xlim([min(self.t_hist), max(self.t_hist)]);
            
            % --- Linear velocities ---
            subplot(2,1,2)
            plot(self.t_hist, self.x_dot_hist(:,4:6), 'LineWidth', 1.2)
            legend('xdot', 'ydot', 'zdot')
            xlabel('Time [s]')
            ylabel('Linear velocity')
            grid on
            xlim([min(self.t_hist), max(self.t_hist)]);
        end
    end
end


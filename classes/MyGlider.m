%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Studio
% MAE5070 - FLIGHT Dynamics
% Treat data for glider dynamics analysis
% Beatriz Asfora
% Mar 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MyGlider

    properties
        % info to id dataset
        gliderID string
        takename string

        % data from Studio, in global coordinates
        timeinput
        posinput
        rotinput

        % gravity (m/s^2)
        g = 9.81;
        
        % time shifted to 0
        time
        
        % origin pt
        pos0G = [0, 0, 0];
        rot0G = [0, 0, 0];
        % origin shift to first data pt
        posT_G
        % initial rotation, DR/G (global to ref)
        DRG

        % cartesian coordinate systems:
        % SG: global, Motion Studio during data collection (inertial)
        posG
        rotG
        
        % SA: Z-down, X, Y parallel to floor (if glider were thrown
        % parallel to the ground)
        DAG
        posA
        rotA

        % SR1: reference, global translated and rotated (inertial);
        DNG
        DNR
        posN
        rotN
        
        % SB: body, solidary to glider at its (approx) center of mass 
        posB
        rotB       
        
        % Z+ direction parallel to gravity, X parallel to floor
        % % global to ref
        % DR_G
        % % ref to body
        % DB_R1
        % % global to body
        % DB_G

        % velocity
        velG
        vN
        vmag

        % to plot
        plotfolder = "plots/"

    end% R1: Z-down, X+ torwards throw
    
    % TODO compute DR_G, DB_R, DB_G, compute velocity
    methods
        
        function obj = source(obj, gliderID, takename)
            % glider name
            obj.gliderID = gliderID;
            obj.takename = takename;
        end

        function obj = new(obj, timearray, positionG, rotationG)

            % save inputs for easy access later
            obj.timeinput = timearray;
            obj.posinput = positionG;
            obj.rotinput =  rotationG;
            
            % make data nicer
            obj = obj.moddata();

            % draft of velocity (finite dif, check with diff or
            % gradiente for discontinuities)
            obj = obj.velDraft();
        end

        function obj = velDraft(obj)
             % velocity draft
            [vel_N, vel_mag] = obj.computeVel(obj.time, obj.posN);
            obj.vN = vel_N;
            obj.vmag = vel_mag;

            % figure;
            % for i =1:3
            %     subplot(1,4,i)
            %     plot(obj.time(2:end), obj.vN(:, i))
            %     hold on
            % end
            % 
            % subplot(1, 4, 4)
            % plot(obj.time(2:end), obj.vmag)
        end

        function obj = moddata(obj)
            %  make data user friendlier

            % move time to start at zero
            obj.time = obj.moveOrigin(obj.timeinput);
            
            % unwrap angles, from -180, 180 to -inf, inf
            obj.rotG = obj.unwrapAngle(obj.rotinput);
            
            % turn from mm to m 
            obj.posG = obj.posinput/1000;

            % ---------------------------------
            % init conditions wrt SG
            % pos
            obj.pos0G = obj.posG(1, :);
            % rot
            obj.rot0G = obj.rotG(1, :);
                       
            % compute DR_G (global to reference)
            seq = 123;
            obj.DRG = obj.getDmatrix(obj.rot0G, seq);

            % compute matrix DN_R (reference to normal)
            rotNR = [90, 0, 0];     % rotate around xR axis 90 degrees 
            obj.DNR = obj.getDmatrix(rotNR, seq);

            % compute matrix DN_G (global to normal)
            obj.DNG = obj.DNR * obj.DRG;
            % ------------------------------------

            % actually modify POSITION data
            %  TRANSLATE origin to where the glider was first seen
            obj.posT_G = obj.moveOrigin(obj.posG);
            
            % ROTATE POSITION data, posN = DNR DRG posG
            obj.posN = obj.rotatebyD(obj.posT_G, obj.DNG);
                       
        end


        % TODO move this to function for all classes
        function plotPos(obj, plotsubfolder, closeopt)
            
            % inputs (just changed to m)
            pos1 = obj.posG;
            % position in SN
            pos2 = obj.posN;
            % ----------------------
            
            % time
            tG = obj.timeinput;
            t = obj.time;

            % handy
            mkrsize1 = 4; 

            % figure
            figure;
            fg = tiledlayout(2, 3);
            
            % plot wrt SG
            for axis=1:3
                if axis == 1
                    axisname = "$X_G$";
                    mycolor = 'r.';
                elseif axis == 2
                    axisname = "$Y_G$";
                    mycolor = 'g.';
                else
                    axisname = "$Z_G$";
                    mycolor = 'b.';
                end

                 nexttile
                 plot(tG, pos1(:, axis), mycolor, 'MarkerSize', mkrsize1)
                 hold on
                 grid on
                xlabel('Time [sec]')
                
                
                yname = "Position " + axisname + " [m]";
                ylabel(yname)
                hold off
            end
            
            % plot wrt SR0
            for axis=1:3
                if axis == 1
                    axisname = "$X_N$";
                elseif axis == 2
                    axisname = "$Y_N$";
                else
                    axisname = "$Z_N$";
                end

                 nexttile
                 plot(t, pos2(:, axis), 'k.', 'MarkerSize', mkrsize1)
                 hold on
                 grid on
                 xlabel('Time [sec]')
                
                
                yname = "Position " + axisname + " [m]";
                ylabel(yname)
                hold off
            end

            
            %% saving
            % where it's gonna be saved
            pfolder = strcat(obj.plotfolder, plotsubfolder);
            % change this for obj.gliderID and add b in the save file
            mytitle = obj.gliderID;
            mysubtitle = strcat(obj.takename, " - Plot C");%, " [visible ", string(obj.pervalid), '%]');
            mysavename = strcat(obj.takename, "C");

            % figure title
            title(fg, mytitle, 'FontSize', 12)
            subtitle(fg,mysubtitle, 'FontSize', 8, 'Interpreter', 'none')

            % save plot as png (todo: change to pdf and crop)
            saveas(gcf,fullfile(pfolder, mysavename), 'png')
            hold off
            
            if closeopt ~= 0
                close
            end

        end
        
        function plotpos3D(obj, plotsubfolder, closeopt)
            % plot data for analysis 
            
            pos1 = obj.posG;
            pos2 = obj.posN;

            t = obj.time;
            % -----------------------------------------

   
            figure;

            % plot body trajectory in 3D
            % subplot(2,2,1)
            fg = tiledlayout(2, 1);

            mkrsize1 = 4; mkrsize2 = 6;

            for i = 1:2
                if i == 1
                    x = pos1(:, 1); y = pos1(:, 2); z = pos1(:, 3);
                    xl = '$X_{G}$ [m]'; yl = "$Y_{G}$ [m]"; zl = "$Z_{G}$ [m]";
                else
                    x = pos2(:, 1); y = pos2(:, 2); z = pos2(:, 3);
                    xl = '$X_{N}$ [m]'; yl = "$Y_{N}$ [m]"; zl = "$Z_{N}$ [m]";
                end

            nexttile;
            plot3(x, y, z, 'b.', 'MarkerSize', mkrsize1)
            if i == 1
                % Y-UP
                camup([0 1 0])
            else
                set(gca, 'ZDir', 'reverse')
            end
            hold on
            % initial collected point
            plot3(x(1), y(1), z(1), 'r*', 'MarkerSize', mkrsize2)
            axis equal
            
            % axis
            xlabel(xl)
            ylabel(yl)
            zlabel(zl)
            % legend
            legend('data pts', 'first pt', 'Location', 'southeast', 'FontSize', 7)
            legend('boxon')
            grid on
            grid minor

            end

            %% saving
            % where it's gonna be saved
            pfolder = strcat(obj.plotfolder, plotsubfolder);
            % change this for obj.gliderID and add b in the save file
            mytitle = obj.gliderID;
            mysubtitle = strcat(obj.takename, " - Plot D");%, " [visible ", string(obj.pervalid), '%]');
            
            mysavename = strcat(obj.takename, "D");

            % figure title
            title(fg, mytitle, 'FontSize', 12)
            subtitle(fg,mysubtitle, 'FontSize', 8, 'Interpreter', 'none')

            % save plot as png (todo: change to pdf and crop)
            saveas(gcf,fullfile(pfolder, mysavename), 'png')
            hold off
            
            if closeopt ~= 0
                close
            end

        end


        function plotData(obj, plotsubfolder, closeopt)
            % plot data for analysis 
            
            pos = obj.posN;

            x = pos(:, 1); 
            y = pos(:, 2); 
            z = pos(:, 3);
            t = obj.time;
            % -----------------------------------------

   
            figure;

            % plot body trajectory in 3D
            % subplot(2,2,1)
            fg = tiledlayout(2, 3);

            mkrsize1 = 4; mkrsize2 = 6;

            nexttile([1 3]);
            plot3(x, y, z, 'b.', 'MarkerSize', mkrsize1)
            % Y-UP
            % camup([0 1 0])
            hold on
            % initial collected point
            plot3(x(1), y(1), z(1), 'r*', 'MarkerSize', mkrsize2)
            set(gca, 'ZDir', 'reverse')
            axis equal
            % axis
            xlabel('$X_{N}$ [m]')
            ylabel('$Y_{N}$ [m]')
            zlabel('$Z_{N}$ [m]')
            % legend
            legend('data pts', 'first pt', 'Location', 'southeast', 'FontSize', 7)
            legend('boxon')
            grid on
            grid minor

            % plot rotation around global axis
            nexttile
            plot(t, x, 'r.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Position $X_N$ [m]')
            hold off

            nexttile
            plot(t, y, 'g.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Position $Y_N$ [m]')
            hold off

            nexttile
            plot(t, z, 'b.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Position $Z_N$ [m]')
            hold off

            %% saving
            % where it's gonna be saved
            pfolder = strcat(obj.plotfolder, plotsubfolder);
            % change this for obj.gliderID and add b in the save file
            mytitle = obj.gliderID;
            mysubtitle = strcat(obj.takename, " - Plot B");%, " [visible ", string(obj.pervalid), '%]');
            
            mysavename = strcat(obj.takename, "B");

            % figure title
            title(fg, mytitle, 'FontSize', 12)
            subtitle(fg,mysubtitle, 'FontSize', 8, 'Interpreter', 'none')

            % save plot as png (todo: change to pdf and crop)
            saveas(gcf,fullfile(pfolder, mysavename), 'png')
            hold off
            
            if closeopt ~= 0
                close
            end

        end

    end

    methods(Static)

        function [velvec, velmag] = computeVel(time, posarray)
            
            x = posarray(:, 1); y = posarray(:, 2); z = posarray(:, 3);

            % finite diference 
            velx = [(x(2:end) - x(1:end-1))./(time(2:end) - time(1:end-1))];
            vely = [(y(2:end) - y(1:end-1))./(time(2:end) - time(1:end-1))]; 
            velz = [(z(2:end) - z(1:end-1))./(time(2:end) - time(1:end-1))];

            velvec = [velx, vely, velz];

            velmag = sqrt( velx.^2 + vely.^2 + velz.^2 ); 

        end

        
        function newpos = rotatebyD(pos, DCM)
            % TODO change this to matrix multiplication (tired now)

            n = size(pos, 1);
            newpos = zeros(n, 3);
            
            for i=1:n
                % DCM(3,3) * pos(3,1) = (3,1)
                vec = DCM * pos(i, :)';
                newpos(i,:) = vec';
            end


        end

        function DCM = getDmatrix(ang, seq)
            % DCM for Euler angle sequence
            
            % radians
            roll = ang(1); pitch = ang(2); yaw = ang(3);
            
            % the XYZ order indicates pitch is degree about the X axis,
            % yaw is degree about the Y axis, and roll is degree about the Z axis
            % pitch = angr(1); yaw = angr(2); roll = angr(3);
            
            % rotation matrices
            % D1(phi)
            D1 = MyGlider.getD1(roll);
            % D2(theta)
            D2 = MyGlider.getD2(pitch);
            % D3(psi)
            D3 = MyGlider.getD3(yaw);
            
            if seq == 123
                DCM = D3 * (D2 * D1);
            elseif seq == 321
                DCM = D1 * (D2 * D3);
            else
                DCM = zeros(3,3);
            end

        end

        function D1 = getD1(phi)
            phi = deg2rad(phi);
            
            D1 = [1, 0, 0; ...
                0, cos(phi), sin(phi); ...
                0, -sin(phi), cos(phi)];
        end

        function D2 = getD2(theta)
            theta = deg2rad(theta);
            
            D2 = [cos(theta), 0, -sin(theta);...
                0, 1, 0; ...
                sin(theta), 0, cos(theta)];
        end

        function D3 = getD3(psi)
            psi = deg2rad(psi);
            
            D3 = [cos(psi), sin(psi), 0;...
                -sin(psi), cos(psi), 0;...
                0, 0, 1];
        end

        function newpos = moveOrigin(posG, pos0G)
           % move origin to where the glider was first seen
           % ie translate according to first data pt 
            
           if ~exist('pos0G', 'var')
                % first position point wrt SG
                pos0G = posG(1, :);
           end

            % translate all positions to be wrt to reference origin
            newpos = posG - pos0G;
        end

        function ang = unwrapAngle(anglearray)
            % unwrap angle from -180, 180 to -inf, inf to avoid
            % discontinuities

            % radians
            ang_rad = deg2rad(anglearray);
            % if there are jumps, replace by complement
            ang_rad_unwr = unwrap(ang_rad);
            % back to degrees
            ang = rad2deg(ang_rad_unwr);

        end

        function DA_G = findDirection(pos)
            % TODO replace by actual DCM >> done, delete
            % change DIRECTION (still with Z+ parallel to gravity)
            % DRG = obj.findDirection(obj.posG);
            
            % direction of axis depending on where the glider is facing
            DA_G = zeros(3, 3);
            
            % number of samples
            n = size(pos,1);
            % get two samples in the middle ish
            x1 = pos(n/3, 1);
            x2 = pos(n/3 + 5, 1);

            % find positive x direction
            if (x2 > x1)
                % throwing neg to pos, keep sign for x 
                DA_G(1, :) = [1 0 0];
                % keep sign, y = z
                DA_G(2, :) = [0 0 1];
            else
                % throwing neg to pos, invert sign for x 
                DA_G(1, :) = [-1 0 0];
                % invert sign, y = -z
                DA_G(2, :) = [0 0 -1];
            end
            % z' = -y' (always, up and down directions)
            DA_G(3, :) = [0 -1 0];
            

        end

    end
end



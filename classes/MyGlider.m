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

        % cartesian coordinate systems:
        % SG: global, Motion Studio during data collection (inertial)
        posN_G
        rot_quat
        rotN_G
        height

        % origin data pt
        posN_Gstart = zeros(1, 3);
        rotN_Gstart = zeros(1, 4);
        % shift SG origin to first data point
        posN_Gtrans

        % SO: (inertial) z-down, x+ direction of movement, origin at the
        % ground where glider was first seen

        posB_O
        % rotation body wrt to SO
        rotB_O

        % ROTATION MATRICES
        % SG to SO (global to zero)
        DO_G = zeros(3,3);

        % to save plot
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
            obj = obj.modifydata();
        end

        function obj = modifydata(obj)
            %  make data user friendlier according to MAE5070 conventions

            % ------------ save inputs using naming convention
            % position: turn mm to m
            obj.posN_G = obj.posinput/1000;
            % height is the measurement in yG direction
            obj.height = obj.posN_G(:, 2);
            % rotation: save (in quaternions)
            obj.rot_quat = obj.rotinput;
            % time: shift to start at zero
            obj.time = obj.moveOrigin(obj.timeinput);
            % ------------

            % starting point: position and rotation of SN wrt to SG
            obj.posN_Gstart = obj.posN_G(1, :);
            obj.rotN_Gstart = obj.rot_quat(1, :);

            % TRANSLATE SG to starting point:
            % [xG, yG, zG] = [xG-xGstart, yG, zG - zGstart]
            deltaStartG = [obj.posN_Gstart(1), 0, obj.posN_Gstart(3)];
            obj.posN_Gtrans = obj.moveOrigin(obj.posN_G, deltaStartG);

            % compute time invariant rotation matrices
            obj.DO_G = obj.findDirection(obj.posN_G);

            % set size up
            n = size(obj.posN_G, 1);
            obj.rotN_G = zeros(n, 3);
            aux_pos = zeros(n, 3);
            rot_body = zeros(n, 3);

            % LOOP through each measurement
            for k=1:n

                % POSITION: from SG to SO at k
                aux_pos(k, :) = obj.changeFrame(obj.posN_Gtrans(k, :), obj.DO_G);

                % ANGLE quaternion (measured)
                qx = obj.rot_quat(k, 1); qy = obj.rot_quat(k, 2); 
                qz = obj.rot_quat(k, 3); qw= obj.rot_quat(k, 4);
                q_array = [qx, qy, qz, qw];

                 % SN wrt SG original (deg)
                [euxG, euyG, euzG] = obj.NatRot(q_array);
                obj.rotN_G(k, :) = [euxG, euyG, euzG];
                
                % get corresponding Tait-Bryan angles SB wrt SO (deg)
                [roll, pitch, yaw] = obj.bodyRot(q_array, obj.DO_G);
                rot_body(k, :) = [roll, pitch, yaw]; 

            end

            % save rot SN wrt SG for checks
            obj.rotN_G = obj.unwrapAngle(obj.rotN_G, false);

            % save rotation of B wrt S0
            obj.rotB_O = obj.unwrapAngle(rot_body, false);

            % save position of glider wrt SO
            obj.posB_O = aux_pos;

        end

       %% PLOT functions
        function plotStudio(obj, plotsubfolder, closeopt)
            % PLOT in one figure
            % (line 1) 3D position wrt to SO, Height instead of true Z
            % (line 2) position vs time (also in S0)
            % (line 3) angles taken from DB/O

            % time
            t = obj.time;

            % position wrt SO
            pos = obj.posB_O;
            x = pos(:, 1); y = pos(:, 2); z = obj.height;

            % rotation SB wrt SO
            rot = obj.rotB_O;

            % aestheticsuntitled3
            mkrsize1 = 4; mkrsize2 = 3;

            % figure
            figure;
            fg = tiledlayout(3, 3);

            % 3D POSITION in SO
            nexttile([1 3]);
            plot3(x, y, z, 'b.', 'MarkerSize', mkrsize1)
            hold on
            % initial collected point
            plot3(x(1), y(1), z(1), 'r*', 'MarkerSize', mkrsize2)
            % axis equal
            set(gca, 'YDir', 'reverse')
            % axis
            xlabel('$X_{O}$ [m]')
            ylabel('$Y_{O}$ [m]')
            zlabel('Height [m]')
            % legend
            legend('data pts', 'first pt', 'Location', 'southeast', 'FontSize', 7)
            legend('boxon')
            grid on
            grid minor
            % [az, el] = view
            view([20, 30])

            % POSITION vs TIME in SO
            axisname = ["${X}^{B/O}_{O}$", "${Y}^{B/O}_{O}$", "${Z}^{B/O}_{O}$"];
            mycolor = ["r.", "m.", "g."];
            for axx=1:3

                nexttile
                if axx == 3
                    % plot height
                    plot(t, z, mycolor(axx), 'MarkerSize', mkrsize1)
                else
                    plot(t, pos(:, axx), mycolor(axx), 'MarkerSize', mkrsize1)
                end
                    hold on
                %if axx == 1
                    yname = "Position [m]";
                    ylabel(yname, 'FontSize',10)
                %end
                % if axx == 3
                %     set(gca, 'ZDir', 'reverse')
                % end
                grid on
                xlabel('Time [sec]')
                title(axisname(axx), 'FontSize',11, 'FontWeight','bold')
                xlim([0 max(t)])
                hold off
            end

            % ANGLE vs TIME wrt S0
            anglename = ["Roll $\phi$", "Pitch $\theta$", "Yaw $\psi$"];
            %anglename = ["Banking $\phi$", "Attack $\theta$", "Heading $\psi$"];
            % mycolor = ["k.", "c.", "y."];
            for axx=1:3

                nexttile
                plot(t, rot(:, axx), mycolor(axx), 'MarkerSize', mkrsize1)
                hold on
                %if axx == 1
                    yname = "Rotation [deg]";
                    ylabel(yname, 'FontSize',10, 'FontWeight','bold')
                %end
                title('Rotation')
                grid on
                xlabel('Time [sec]')
                title(anglename(axx), 'FontSize',12)
                xlim([0 max(t)])
                hold off
            end

            %% saving
            % where it's gonna be saved
            pfolder = strcat(obj.plotfolder, plotsubfolder);
            % change this for obj.gliderID and add b in the save file
            mytitle = obj.gliderID + " - " + "Body Data";
            mysubtitle = strcat(obj.takename);%, " [visible ", string(obj.pervalid), '%]');
            mysavename = strcat(obj.takename, "_main");

            % figure title
            title(fg, mytitle, 'FontSize', 12, 'Interpreter', 'none')
            subtitle(fg,mysubtitle, 'FontSize', 8, 'Interpreter', 'none')

            % save plot as png 
            saveas(gcf,fullfile(pfolder, mysavename), 'png')
            
            % save plot as matlab fig
            saveas(gcf, fullfile(pfolder, mysavename), 'fig')

            hold off

            if closeopt ~= 0
                close
            end

        end

        function plotOriginal(obj, plotsubfolder, closeopt)
            % PLOT in one figure
            % (line 1) 3D position wrt to SG, Height instead of true Z
            % (line 2) position vs time (also in SG)
            % (line 3) angles wrt to SG

            % time
            t = obj.time;

            % position wrt SG
            pos = obj.posN_G;
            x = pos(:, 1); y = pos(:, 2); z = pos(:, 3);

            % rotation SN wrt SG
            rot = obj.rotN_G;

            % aestheticsuntitled3
            mkrsize1 = 4; mkrsize2 = 3;

            % figure
            figure;
            fg = tiledlayout(3, 3);

            % 3D POSITION in SO
            nexttile([1 3]);
            plot3(x, y, z, 'b.', 'MarkerSize', mkrsize1)
            hold on
            % initial collected point
            plot3(x(1), y(1), z(1), 'r*', 'MarkerSize', mkrsize2)
            % axis equal
            % set(gca, 'YDir', 'reverse')
            % Y-UP
            camup([0 1 0])
            % axis
            xlabel('$X_{G}$ [m]')
            ylabel('$Y_{G}$ [m]')
            zlabel('$Z_{G}$ [m]')
            % legend
            legend('data pts', 'first pt', 'Location', 'southeast', 'FontSize', 7)
            legend('boxon')
            grid on
            grid minor
            % [az, el] = view
            view([20, 30])

            % POSITION vs TIME in SO
            axisname = ["$X^{N/G}_{G}$", "$Y^{N/G}_{G}$", "$Z^{N/G}_{G}$"];
            mycolor = ["r.", "m.", "g."];
            for axx=1:3

                nexttile
                plot(t, pos(:, axx), mycolor(axx), 'MarkerSize', mkrsize1)
                hold on
                title('Position')
                grid on
                xlabel('Time [sec]')
                yname =  axisname(axx) + " [m]";
                ylabel(yname)
                xlim([0 max(t)])
                hold off
            end

            % ANGLE vs TIME wrt S0
            anglename = axisname;
            mycolor = ["k.", "c.", "y."];
            for axx=1:3

                nexttile
                plot(t, rot(:, axx), mycolor(axx), 'MarkerSize', mkrsize1)
                hold on
                title('Rotation')
                grid on
                xlabel('Time [sec]')
                yname =  anglename(axx) + " [deg]";
                ylabel(yname)
                xlim([0 max(t)])
                hold off
            end

            %% saving
            % where it's gonna be saved
            pfolder = strcat(obj.plotfolder, plotsubfolder);
            % change this for obj.gliderID and add b in the save file
            mytitle = obj.gliderID + " - " + "Global Data";
            mysubtitle = strcat(obj.takename);%, " [visible ", string(obj.pervalid), '%]');
            mysavename = strcat(obj.takename, "_original");

            % figure title
            title(fg, mytitle, 'FontSize', 12, 'Interpreter', 'none')
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

        function [roll, pitch, yaw] = bodyRot(quat_array, DOG)
            % turns:
            % measured quaternion rotation (from Optitrack) wrt SG
            % into Tait–Bryan angles, sequence 'ZYX'wrt SO
            % INPUTS:
            % quat_array = [qx, qy, qz, qw]
            % DOG : DCM [3x3] for SG to SO
            % OUTPUT:
            % roll, pitch, yaw in DEG

            % Adapted from NatNet SDK 4.1.1 > Samples > MATLAB
            [eulerx, eulery, eulerz] = MyGlider.NatRot(quat_array);
            
            % use MAE5070 conventions
            % roll OK | pitch | yaw
            if DOG(1,1) < 0
                % throw from negative to positive
                roll =  -eulerx - 180;  
                pitch = 180 -(eulerz);
                yaw = eulery;
            else
                roll = eulerx; % xB = xG (OK)
                pitch = -eulerz;
                yaw = -eulery;
            end   

        end

        function [eulerx, eulery, eulerz] = NatRot(quat_array)
            qx = quat_array(1); qy = quat_array(2); 
            qz = quat_array(3); qw = quat_array(4);
            % From NatNet SDK 4.1.1 > Samples > MATLAB
            q = quaternion( qx, qy, qz, qw );
            qRot = quaternion( 0, 0, 0, 1);
            q = mtimes( q, qRot);
            a = EulerAngles( q , 'zyx' );
            % change to degrees
            eulerx = a( 1 ) * -180.0 / pi;
            eulery = a( 2 ) * 180.0 / pi;
            eulerz = a( 3 ) * -180.0 / pi;

        end


        function vec_S2 = changeFrame(vec_S1, DCM)
            % INPUTS: pos_S1 [1x3]; DCM = 3x3
            % OUTPUT: pos_S2 [1x3];
            column_vec = DCM * vec_S1';
            vec_S2 = column_vec';

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
        
        function ang = unwrapAngle(anglearray, rad)
            % unwrap angle from -180, 180 to -inf, inf to avoid
            % discontinuities
            
            if ~rad
                % radians
                ang_rad = deg2rad(anglearray);
            else
                ang_rad = anglearray;
            end

            dim = size(anglearray, 2);
            % increase allocation speed
            new_ang = ang_rad;

            for k=1:dim
                % if there are jumps, replace by complement
                new_ang(:, k) = unwrap(ang_rad(:, k));
            end

            % back to degrees
            ang = rad2deg(new_ang);

        end

        function DA_G = findDirection(pos)
            % change DIRECTION (still with Z+ parallel to gravity)
            % DRG = obj.findDirection(obj.posG);

            % direction of axis depending on where the glider is facing
            DA_G = zeros(3, 3);

            % number of samples
            n = size(pos,1);
            % get two samples in the middle ish
            pt = round(n/3);
            x1 = pos(pt, 1);
            x2 = pos(pt + 5, 1);

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

        function DCM = getDmatrix(ang, seq)
            % DCM for Euler angle sequence

            % radians
            roll = ang(1); pitch = ang(2); yaw = ang(3);

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
            % Motive doc: the XYZ order indicates pitch is degree about the X axis,
            % yaw is degree about the Y axis, and roll is degree about the Z axis
            % pitch = angr(1); yaw = angr(2); roll = angr(3);
        end

        function D1 = getD1(phi)
            
            D1 = [1, 0, 0; ...
                0, cos(phi), sin(phi); ...
                0, -sin(phi), cos(phi)];
        end

        function D2 = getD2(theta)

            D2 = [cos(theta), 0, -sin(theta);...
                0, 1, 0; ...
                sin(theta), 0, cos(theta)];
        end

        function D3 = getD3(psi)

            D3 = [cos(psi), sin(psi), 0;...
                -sin(psi), cos(psi), 0;...
                0, 0, 1];
        end

    end
end


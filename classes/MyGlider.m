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
        rotN_G
        height

        % origin data pt
        posN_Gstart = zeros(1, 3);
        rotN_Gstart = zeros(1, 3);
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
        % SO to SG (zero to global) 
        DG_O = zeros(3,3);
        % SG to SN (global to nominal)
        DN_G = zeros(3,3);
        % SN to SB (nominal to body)
        DB_N = zeros(3,3);        

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

            % draft of velocity (finite dif, check with diff or
            % gradiente for discontinuities)
            % obj = obj.velDraft();
        end
    
        function obj = modifydata(obj)
            %  make data user friendlier according to MAE5070 conventions
            
            % ------------ save inputs using naming convention
            % position: turn mm to m
            obj.posN_G = obj.posinput/1000;
            % height is the measurement in yG direction
            obj.height = obj.posN_G(:, 2);
            % rotation: save
            obj.rotN_G = obj.rotinput;
            % time: shift to start at zero
            obj.time = obj.moveOrigin(obj.timeinput);
            % ------------

            % starting point: position and rotation of SN wrt to SG
            obj.posN_Gstart = obj.posN_G(1, :);
            obj.rotN_Gstart = obj.rotN_G(1, :);
            
            % TRANSLATE SG to starting point: 
            % [xG, yG, zG] = [xG-xGstart, yG, zG - zGstart]
            deltaStartG = [obj.posN_Gstart(1), 0, obj.posN_Gstart(3)];
            obj.posN_Gtrans = obj.moveOrigin(obj.posN_G, deltaStartG);
            
            % compute time invariant rotation matrices
            obj = obj.computeKnownDs();

            % set size up
            n = size(obj.posN_G, 1);
            aux_pos = zeros(n, 3);
            aux_rot = zeros(n, 3);
            
            % LOOP through each measurement
            for k=1:n
                
                % POSITION: from SG to SO at k
                aux_pos(k, :) = obj.changeFrame(obj.posN_Gtrans(k, :), obj.DO_G);

                % ANGLE: we want SB wrt S0 at k
                aux_rot(k, :) = obj.computeEuler(k);

            end

            % save position of glider wrt SO
            obj.posB_O = aux_pos;

            % save rotation of B wrt S0
            obj.rotB_O = rad2deg(unwrap(aux_rot));

        end

        function eul_angles = computeEuler(obj, k)
            % OUTPUT: new Euler Angles in radians

            % just for visibility
            DBN = obj.DB_N; DGO = obj.DG_O;


            % current measured angles (from Motive software)
                phi = obj.rotN_G(k, 1);
                theta = obj.rotN_G(k, 3);
                yaw = obj.rotN_G(k, 2);

                % current rotation matrix (DCM)
                xyz = [phi, theta, yaw]; seq = 123;
                DNG_k = obj.getDmatrix(xyz, seq);

                % compute DB/0 (rot SB from S0)
                DBO_k = DBN * DNG_k * DGO;

                % angles of interest (rotation of body wrt to SO)
                eul_angles = rotm2eul(DBO_k, 'XYZ');

        end


        function obj = computeKnownDs(obj)

            % ROTATION MATRICES            
            % SG to SO (global to zero) - rotate 90 deg on xG
            obj.DO_G = obj.findDirection(obj.posN_G);
            % SO to SG (zero to global) -- transpose of DO_G
            obj.DG_O = transpose(obj.DO_G);
            % SN to SB (nominal to body) - rotate 90 deg on xN
            obj.DB_N = obj.DO_G;

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
            axisname = ["$X_O$", "$Y_O$", "$Z_O$"];
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
            anglename = ["$\phi$", "$\theta$", "$\psi$"];
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
            mytitle = obj.gliderID + " - " + "Body Data";
            mysubtitle = strcat(obj.takename);%, " [visible ", string(obj.pervalid), '%]');
            mysavename = strcat(obj.takename, "main");

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
        
        function plotOriginal(obj, plotsubfolder, closeopt)
            % PLOT in one figure
            % (line 1) 3D position wrt to SO, Height instead of true Z
            % (line 2) position vs time (also in S0)
            % (line 3) angles taken from DB/O
            
            % time
            t = obj.time;
            
            % position wrt SO
            pos = obj.posN_G;
            x = pos(:, 1); y = pos(:, 2); z = pos(:, 3);
            
            % rotation SB wrt SO
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
            axisname = ["$X_G$", "$Y_G$", "$Z_G$"];
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
            mysavename = strcat(obj.takename, "original");

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

        % TODO move this to function for all classes
        function plotPos(obj, plotsubfolder, closeopt)

            % inputs (just changed to m)
            pos1 = obj.posN_G;
            % position in SO
            pos2 = obj.posB_O;
            x = pos2(:, 1); 
            y = pos2(:, 2); 
            z = obj.height;
            % ----------------------

            % time
            tG = obj.timeinput;
            t = obj.time;

            % handy
            mkrsize1 = 4; mkrsize2 = 6;

            % figure
            figure;
            fg = tiledlayout(3, 3);

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
            

            % plot wrt SR0
            for axx=1:3
                if axx == 1
                    axisname = "$X_O$";
                elseif axx == 2
                    axisname = "$Y_O$";
                else
                    axisname = "$Z_O$";
                end

                nexttile
                plot(t, pos2(:, axx), 'k.', 'MarkerSize', mkrsize1)
                hold on
                grid on
                xlabel('Time [sec]')


                yname = "Position " + axisname + " [m]";
                ylabel(yname)
                hold off
            end

            % plot wrt SG
            for axx=1:3
                if axx == 1
                    axisname = "$X_G$";
                    mycolor = 'r.';
                elseif axx == 2
                    axisname = "$Y_G$";
                    mycolor = 'g.';
                else
                    axisname = "$Z_G$";
                    mycolor = 'b.';
                end

                nexttile
                plot(tG, pos1(:, axx), mycolor, 'MarkerSize', mkrsize1)
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
            mytitle = obj.gliderID + 'Global data';
            mysubtitle = strcat(obj.takename);%, " [visible ", string(obj.pervalid), '%]');
            mysavename = strcat(obj.takename, "dataG");

            % figure title
            title(fg, mytitle, 'FontSize', 12)
            subtitle(fg,mysubtitle, 'FontSize', 8, 'Interpreter', 'none')

            % save plot as png (todo: change to pdf and crop)
            saveas(gcf,fullfile(pfolder, mysavename), 'png')
            hold off

            if closeopt ~= 0
                close
            end

                        %%
            % figure;
            % plot(obj.time, obj.posG(:, 2), 'r')
            % hold on
            % %plot(obj.time, obj.height, 'b--')
            % plot(obj.time, -obj.posSO(:, 3), 'k')
            % title('Height Check')
            % legend('Y-UP (raw)', '- $Z_O$', 'Interpreter', 'latex')

        end

        function plotpos3D(obj, plotsubfolder, closeopt)
            % plot data for analysis

            pos1 = obj.posN_G;
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
        
    end

    methods(Static)

        function vec_S2 = changeFrame(vec_S1, DCM)
            % INPUTS: pos_S1 [1x3]; DCM = 3x3
            % OUTPUT: pos_S2 [1x3];
            column_vec = DCM * vec_S1';
            vec_S2 = column_vec';

        end

        function [velvec, velmag] = computeVel(time, posarray)

            x = posarray(:, 1); y = posarray(:, 2); z = posarray(:, 3);

            % finite diference
            velx = (x(2:end) - x(1:end-1))./(time(2:end) - time(1:end-1));
            vely = (y(2:end) - y(1:end-1))./(time(2:end) - time(1:end-1));
            velz = (z(2:end) - z(1:end-1))./(time(2:end) - time(1:end-1));

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
            pt = round(n/2);
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

    end
end


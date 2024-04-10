%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Studio 
% MAE5070 - FLIGHT Dynamics
% Import data from single csv file
% Beatriz Asfora
% Mar 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MyTake
    % import data from a single csv file
    % run checks to get rid of gaps (no interpolation)

    % return tracking data: time and 6DoF rigid body pose in global coordinates
    % wrt to Motion Studio origin

    properties
        % for each take
        filename string
        name string
        filepath string
        fullpath string

        % raw data
        dataraw
        n {mustBeInteger}
        bodyname string
        allframes
        alltimes
        allposes
        allmarkers
        allangles

        % logical for which data to keep
        tokeep logical

        % useful data info to return
        time
        frames
        trackingdata
        rotationdata
        positiondata
        goodn {mustBeInteger}
        pervalid

        % to plot
        plotfolder = "plots/"
        commoname string


    end

    methods

        function obj = new(obj, filename, filepath)

            % save
            obj.filepath = filepath;
            obj.filename = filename;
            obj.fullpath = strcat(filepath, filename);

            % title for plot
            obj.name = erase(filename,".csv");
            disp(strcat("Take: ", obj.name))
            % name of rigid body
            obj.bodyname = obj.headerRow(obj.fullpath, 1);

            % TODO remove later
            if (contains(obj.bodyname, 'Pink') || contains(obj.bodyname, 'V2Glider'))
                altname = 'V3Glider';
                obj.commoname = altname;
            else
                obj.commoname = obj.bodyname;

            end
            %

            disp(strcat("Rigid body: ", obj.bodyname, "[", obj.commoname,"]"))

            % import raw data
            obj.dataraw = obj.importTrackingData(obj.fullpath);

            % run some checks, discard some frames
            obj = obj.checkData();

        end

        function obj = checkData(obj)

            datatake = obj.dataraw;
            obj.n = size(datatake, 1);
            disp("Frames")
            disp(strcat("... collected: ", string(obj.n)))

            % separate raw data
            rw1 = 1; col1 = 3; col2 = 9;
            % column 1: frame numbers
            obj.allframes = datatake(rw1:end, 1);
            % column 2: time stamp
            obj.alltimes = datatake(rw1:end, 2);
            % columns 3-6: XYZ rotation, XYZ position
            obj.allposes = datatake(rw1:end,col1:col1+5);
            % columns 7-end: rigid body markers
            obj.allmarkers = datatake(rw1:end, col2:end);

            %% what we consider valid

            % not empty lines (NaN)
            isNumerical = ~isnan(obj.allposes(:,1));

            % not zero values (0, 0, 0, 0, 0, 0)
            isNz = sum(obj.allposes, 2) ~=0;

            % not all markers are occluded
            isVis = ~all(isnan(obj.allmarkers), 2);

            % 0 for drop, 1 to keep
            rowstokeep = isNumerical & isNz & isVis;

            % take invalid data out and save
            obj.time = obj.alltimes(rowstokeep, :);
            obj.frames = obj.allframes(rowstokeep, :);
            obj.trackingdata = obj.allposes(rowstokeep, :);
            
            % angles (unwrap)
            obj.rotationdata = obj.unwrapAngle(obj.trackingdata(:, 1:3));
            % position
            obj.positiondata = obj.trackingdata(:, 4:6);
            obj.goodn = size(obj.trackingdata, 1);
            obj.tokeep = rowstokeep;
            % id when exp ended
            endexp = obj.frames(end, 1) + 1;
            obj.pervalid = ceil((obj.goodn / endexp) * 100);

            % print data checks results
            disp(strcat("... for analysis: ", string(obj.goodn)))
            disp(strcat("... valid %: ", string(obj.pervalid)))
            disp('----------------------')

        end

        %% PLOT and retreieve functions

        function [timearray, positionarray, rotationarray] = getData(obj)
            % return time, rotation (XYZ) and position (XYZ) in global
            % cordinates
            timearray = obj.time;
            positionarray = obj.positiondata;
            rotationarray = obj.rotationdata;

        end

        function plotData(obj, plotsubfolder, closeopt)
            % plot data for analysis in global coordinates
            % scatter plot (no interpolation between missing data)

            xG = obj.positiondata(:, 1)/1000; yG = obj.positiondata(:, 2)/1000; zG = obj.positiondata(:, 3)/1000;
            rotxG = obj.rotationdata(:, 1); rotyG = obj.rotationdata(:, 2); rotzG = obj.rotationdata(:, 3);
            t = obj.time;

            % where it's gonna be saved
            obj.plotfolder = strcat(obj.plotfolder, plotsubfolder);

            figure;

            % plot body trajectory in 3D
            % subplot(2,2,1)
            fg = tiledlayout(2, 3);

            mkrsize1 = 4; mkrsize2 = 6;
            lnwdth = 0.6;

            nexttile([1 3]);
            plot3(xG, yG, zG, 'b.', 'MarkerSize', mkrsize1)
            % Y-UP
            camup([0 1 0])
            hold on
            % initial collected point
            plot3(xG(1), yG(1), zG(1), 'r*', 'MarkerSize', mkrsize2)
            ax = gca;
            dx = xlim; dy = ylim; dz = zlim;
            % xrange = dx(2) - dx(1);
            % yrange = dy(2) - dy(1);
            % zrange = dz(2) - dz(1);
            % title('Body Trajectory')
            % second line or text: take name
            % axis
            xlabel('$X_G$ [m]')
            ylabel('$Y_G$ [m]')
            zlabel('$Z_G$ [m]')
            % move z axis to bottom of plot
            % ax.ZRuler.FirstCrossoverValue  =1; % Z crossover with X axis
            % ax.ZRuler.SecondCrossoverValue = 0; % Z crossover with Y axis
            % ax.XRuler.SecondCrossoverValue = dx(2); % X crossover with Z axis
            % ax.YRuler.SecondCrossoverValue = dy(1); % Y crossover with Z axis
            % legend
            legend('data pts', 'first pt', 'Location', 'southeast', 'FontSize', 7)
            legend('boxon')
            grid on
            grid minor

            % plot rotation around global axis
            %subplot(2, 2, 2)
            nexttile
            plot(t, rotxG, 'r.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Rotation $X_G$ [deg]')
            hold off

            %subplot(2, 2, 3)
            nexttile
            plot(t, rotyG, 'g.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Rotation $Y_G$ [deg]')
            hold off

            % subplot(2, 2, 4)
            nexttile
            plot(t, rotzG, 'b.', 'MarkerSize',mkrsize1)
            hold on
            grid on
            xlabel('Time [sec]')
            ylabel('Rotation $Z_G$ [deg]')
            hold off

            % TODO remove later
            if (contains(obj.bodyname, 'Pink') || contains(obj.bodyname, 'V2Glider'))
                mytitle = strcat(obj.commoname, " [", obj.bodyname, "]", "- Global Data");
            else
                mytitle = obj.bodyname + " - Global Data";
            end
            %

            mysubtitle = [strcat(obj.name, ...
                " [visible ", string(obj.pervalid), '%]')];

            % figure title
            title(fg, mytitle, 'FontSize', 12)
            subtitle(fg,mysubtitle, 'FontSize', 9, 'Interpreter', 'none');
            mysavename = strcat(obj.name, 'original');


            % save plot as png (todo: change to pdf and crop)
            saveas(gcf,fullfile(obj.plotfolder, mysavename), 'png')
            hold off

            if closeopt ~= 0
                close
            end

        end

    end

    methods(Static)

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

        function bodyname = headerRow(rel_path, k)
            % Read the header row
            fid = fopen(rel_path, 'r');
            % rigid bodies names
            text_row = textscan(fid, '%s', 1, 'Delimiter', '\n', 'HeaderLines', 3);
            % measurements header
            header_row = textscan(fid, '%s', 1, 'Delimiter', '\n', 'HeaderLines', 2);
            fclose(fid);

            % Extract column names from the header
            header = split(header_row{1}{1}, ',');
            % Extract text from the 3rd column onwards
            text_cell = split(text_row{1}{1}, ',');
            body_names = text_cell(3:end);

            if k == 1
                bodyname = body_names(1);
            end

            % Displaying the header
            %disp('Header:');
            %disp(header);

            % Display the extracted text
            %disp('Rigid Bodies');
            %disp(body_names);
        end

        function data = importTrackingData(rel_path)
            % import actual data from csv file, starting from 8th row
            data = readmatrix(rel_path, 'Range','A8');
        end

    end



end

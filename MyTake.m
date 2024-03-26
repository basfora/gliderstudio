classdef MyTake

    properties
        % for each take
        filename string
        name string
        filepath string
        fullpath string

        % data
        dataraw
        n {mustBeInteger}
        bodyname string
        allframes
        alltimes

        datavalid
        validn {mustBeInteger}
        
        time
        frames
        trackingdata


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
            disp(strcat("Rigid body: ", obj.bodyname))

            % import raw data
            obj.dataraw = obj.importTrackingData(obj.fullpath);

            % run some checks
            obj = obj.checkData();
        end

        function obj = checkData(obj)

            datatake = obj.dataraw;
            obj.n = size(datatake, 1);
            disp(strcat("Frames collected: ", string(obj.n)))
            
            % create variables
            rw1 = 1; col1 = 3;
            obj.allframes = datatake(rw1:end, 1);
            obj.alltimes = datatake(rw1:end, 2);

            % data from tracked bodies 
            % databody = zeros(size(frames,1),6);
            databody = datatake(rw1:end,col1:col1+5);

            % get rid of empty lines
            isNumerical = ~isnan(databody(:,1));
            validdata = databody(isNumerical,:);
            % get rid of 0 lines (old motive format, might be unecessary)
            isNz =  (~validdata(:,1)==0);
            % del lines that are repeated?
            % NaN (not-a-number) values
            % dat = dat(~isnan(sum(dat,2)),:);
            % remove zeros
            % dat = dat(sum(dat(:,6:8),2) ~= 0,:);


            % valid data 
            obj.datavalid = validdata(isNz,:);
            obj.validn = size(obj.datavalid, 1);

            % position and orientation of body only
            obj.trackingdata = obj.datavalid(:, 1:6);

            obj.frames = obj.allframes(isNumerical, :);
            obj.time = obj.alltimes(isNumerical, :);

            obj.frames = obj.frames(isNz,:);
            obj.time = obj.time(isNz,:);

            disp(strcat('Frames useful: ', string(obj.validn)))

        end

    end

    methods(Static)

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

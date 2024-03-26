classdef DataBatch

    properties
        % location of csv file, data folder
        datafolder string
        % partticular collection session
        sessionfolder string
        folderpath string
        % date of collection
        dateCollected string

        files struct
        nFiles {mustBeInteger}

        array_filenames = []
        array_full_path = []

        % implement later (maybe) -- dependent on folder name format 
        % array_name struct
        % array_rawdata struct
        % array_IDs struct
        

    end

    methods

        function [obj, filename, filepath] = setPaths(obj)
    
            obj.array_filenames = strings(obj.nFiles, 1);
            obj.array_full_path = strings(obj.nFiles, 1);

            for k = 1:obj.nFiles
            % name of the file, e.g. DIS201_take01
            filename = obj.files(k).name;
            filepath = obj.folderpath;
            % path to find it
            full_path = strcat(obj.folderpath, filename);
            
            % store
            obj.array_filenames(k) = filename;
            obj.array_full_path(k) = full_path;

            end
        end
        

        function obj = newBatch(obj, datafolder, sessionfolder)
            % parent folder where datasets are being saved
            obj.datafolder = datafolder;
            % modify here for different session folder
            obj = obj.setInfo(sessionfolder);
            % import files
            obj = obj.importFiles();
            % save filenames and paths to recover 
            obj = obj.setPaths();
        end

        function obj = importFiles(obj)
            % set complete folder name
            obj.folderpath = strcat(obj.datafolder, obj.sessionfolder,'/');
            % import csv files only
            obj.files = dir(fullfile(obj.folderpath,'*.csv'));
            obj.nFiles = size(obj.files, 1);
            % print number of files
            obj.dispNumberFiles()
        end

        function obj = setInfo(obj, foldername)
            % get info about collected data
            obj.sessionfolder = foldername;

            % change this if way of naming folder changes
            aux = split(foldername,'-');
            obj.dateCollected = aux(1);
            % print
            obj.dispInit()
        end

        function dispInit(obj)
            % PRINT function
            disp(strcat("Processing data from folder... ", obj.sessionfolder))
            disp('')
        end

        function dispNumberFiles(obj)
            % PRINT function
            disp(strcat("Datasets found... ", string(obj.nFiles)))
            disp("----------------------------------------------")

        end


    end


end
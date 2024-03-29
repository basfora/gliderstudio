 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Studio 
% MAE5070 - FLIGHT Dynamics
% Data check and plot
% Beatriz Asfora
% Mar 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
clc
close all
set(0,'defaulttextinterpreter','latex')
warning('off','all')

% folder where csv files are saved
myfolder = "okdata";
% 0308, 0311, 0313, 0325, 

% create obj to use data methods
mysession = MyBatch;
mysession = mysession.newBatch("trackingData/", myfolder);

% trackingData/myfolder/"
fpath = mysession.folderpath;

for k = 1:mysession.nFiles
    
    % name of the file within session folder
    fname = mysession.array_filenames(k);
    
    % new take obj
    take = MyTake;
    % collect data and runs checks
    take = take.new(fname, fpath);

    % get data for analysis
    [time, position, rotation] = take.getData;
    
    % plot tracking data in global coordinates
    take.plotData();


end

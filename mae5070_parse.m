 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Studio 
% MAE5070 - FLIGHT Dynamics
% Data check and plot
% Beatriz Asfora
% Mar 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add this command so MATLAB can find the necessary functions
addpath classes/
% 
clear, clc, close all
set(0,'defaulttextinterpreter','latex')
warning('off','all')


% folder where csv files are saved
myfolder = "data60";

% create obj to use class methods
mysession = MyBatch;
mysession = mysession.newBatch("trackingData/", myfolder);

% trackingData/myfolder/"
fpath = mysession.folderpath;

for k = 1:mysession.nFiles
    
    % name of the file within session folder
    fname = mysession.getfname(k);
    
    % new take obj
    take = MyTake;
    % collect data and runs checks
    take = take.new(fname, fpath);

    % get data for analysis
    [time, position, rotation] = take.getData;
    
    % plot tracking data in global coordinates
    plotfoldername = "takePlots";
    take.plotData(plotfoldername);


end

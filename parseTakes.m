 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motion Studio 
% MAE5070 - FLIGHT Dynamics
% Data check and plot
% Beatriz Asfora
% Mar 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add this command so MATLAB can find the necessary functions
addpath classes/
% handy commands
clear, clc, close all
set(0,'defaulttextinterpreter','latex')
warning('off','all')

% ------------------------------------------
% change folder name here
% folder where csv files are saved
sessionfolder = "data60";
% ------------------------------------------

% create obj to use class methods
session = MyBatch;
session = session.newBatch("data/", sessionfolder);

% trackingData/myfolder/"
fpath = session.folderpath;

for k = 1:session.nFiles
    
    % name of the file within session folder
    fname = session.getfname(k);
    
    % new take obj
    take = MyTake;
    % collect data and runs checks
    take = take.new(fname, fpath);

    % ----------------
    % get data for analysis
    [time, positionG, rotationG] = take.getData;
    % plot tracking data in global coordinates
    plotfoldername = strcat("plots", "_", sessionfolder);
    % plot
    closeplot = 0;
    take.plotData(plotfoldername, closeplot);
    % -----------------
    
    % new glider obj
    glider = MyGlider;
    glider = glider.new(time, positionG, rotationG);

    % input glidername and takename for easy access
    glider = glider.source(take.commoname, take.name);
    
    %%  just to check
    glider.plotPos(plotfoldername, closeplot)
    glider.plotData(plotfoldername, closeplot)

end




















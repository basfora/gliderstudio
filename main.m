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
% save files inside mydata
sessionfolder = "mydata";
% close figures after saving them; 1 to close, 0 to leave them open
closeplot = 0;
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
    take.plotData(plotfoldername, closeplot);
    % -----------------
    
    % new glider obj
    glider = MyGlider;
    glider = glider.new(time, positionG, rotationG);

    % input glidername and takename for easy access
    glider = glider.source(take.commoname, take.name);
    
    %  plot
    glider.plotStudio(plotfoldername, closeplot)

    %% Glider data 

    % position wrt SO
    positionB_SO = glider.posB_O;

    % rotation body wrt SO
    rotationB_SO = glider.rotB_O;

    % height
    height = glider.height;

    % save
    save(glider.takename, "height", "rotationB_SO", "positionB_SO")

end









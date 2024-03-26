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


% create obj to use data methods
mysession = DataBatch;
mysession = mysession.newBatch("trackingData/", "0313-V2");

fpath = mysession.folderpath;

for k = 1:mysession.nFiles
    
    % get name of the file within session folder
    fname = mysession.array_filenames(k);
    
    % new take obj
    take = MyTake;

    % collect data and runs checks
    take = take.new(fname, fpath);

    continue
    
    %% STOPPED HERE

   
    %% get data GLIDER SPECIFIC
    % check RB names (MASS, CART) and order
    if strcmp(body_names(1), rb1)
        % mass_pose = (n, 3); 
        pose_mass = datavalid(:, 1:3);
        % cart_pose = zeros(n, 3);
        pose_cart = datavalid(:, 4:6);
    else
        pose_cart = datavalid(:, 1:3);
        pose_mass = datavalid(:, 4:6);
    end  

    % mass
    xmass = pose_mass(:, 1); ymass = pose_mass(:, 2); zmass = pose_mass(:, 3);
    % cart
    xcart = pose_cart(:, 1); ycart = pose_cart(:, 2); zcart = pose_cart(:, 3);
   
    %% plot MASS trajectory
    % plot 3D time vs x,y,z
    subplot(2, 1, 1)
    plot3(xmass, ymass, zmass, 'b')
    % Y-UP
    camup([0 1 0])
    hold on
    % initial condition mass
    plot3(xmass(1), ymass(1), (zmass(1)), 'b*', 'LineWidth',4) 
    % axis equal
    % title
    dx = xlim; dy = ylim; dz = zlim;
    xrange = dx(2) - dx(1);
    yrange = dy(2) - dy(1);
    zrange = dz(2) - dz(1);
    pos_title = [dx(1)+0.8*xrange, dy(1) + 0.9*yrange, dz(1) + 0.9*zrange];
    title(take_name, 'Position', pos_title)
    % axis
    xlabel('X [mm]')
    ylabel('Y [mm]')
    zlabel('Z [mm]')
    
    % legend
    legend('MASS', 'mass at t = 0')
    legend('boxon')
    grid on
    
    %% plot CART trajectory
    subplot(2, 1, 2)
    plot(xcart, zcart, 'm')
    hold on
    % initial condition cart
    plot(xcart(1), zcart(1), 'm^', 'LineWidth',4)
    grid on 
    p = 500;
    xlim([dx(1)-p dx(2)+p])
    ylim([dz(1)-p dz(2)+p])

    legend('CART', 'cart at t = 0')
    legend('boxon')
    xlabel('X [mm]')
    ylabel('Z [mm]')
    

    % save plot inside folder
   %saveas(gcf,take_name, 'jpg')
    % for file without section name uncomment the line below
   saveas(gcf,strcat(section,'_',take_name), 'jpg')
   hold off
   close

end

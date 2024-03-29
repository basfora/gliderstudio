classdef GliderData

    properties
        gliderID string
        name string
        
        % data from Studio, in global coordinates
        timedata
        positionG
        rotationG

        % gravity (m/s^2)
        g = 9.81;

        % cartesian coordinate systems:
        % SG: global, Motion Studio during data collection
        % SB: body, solidary to glider at its center of mass (approx)
        % SR: reference, initial frame aligned with body 

        % global to ref
        DR_G
        % ref to body        
        DB_R
        % global to body
        DB_G
                
    end

    methods
        

        % compute DR_G, DB_R, DB_G

        % take out time

        % compute velocity
        
    end

end
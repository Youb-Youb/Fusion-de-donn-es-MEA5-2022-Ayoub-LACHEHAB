function [AngleEuler]=eulerIntegration(t,dt,Angular_velocity)

        AngleEuler =1/2*(Angular_velocity)*t^2+W0; % sch�ma num�rique    
end

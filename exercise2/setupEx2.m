function [prob, opt] = setupEx2(dim, speed, seed)
%
% Setup a random quadratic program
%

%% Create a random quadratic program with dim variables
prob = randomQP(dim, seed);

%% Plot the problem
if dim == 2
  plotOptimization(prob);
end

%% Barrier method parameters
% Select 'true' for slow convergence, so you can see the central path.
% Select 'false' for standard, fast convergence
opt = setBarrierParameters(strncmp(speed, 'slow', 1));

end
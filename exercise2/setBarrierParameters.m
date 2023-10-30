
%% Set the optimization parameters
function opt = setBarrierParameters(isSlow)
if isSlow
  % Use these parameters for "slow" convergence, so you can see the central path
  opt.kappa0  = 100;   % Initial value of barrier parameter
  opt.alpha   = 0.02;  % Line search parameters
  opt.beta    = 0.9;
  opt.mu      = 0.8;   % Decrease rate for barrier parameter
  opt.epsilon = 1e-3;  % Optimality tolerance
  opt.slow    = true;
else
  % Use these parameters for "fast" convergence, to solve the optimization problem
  opt.kappa0  = 100;
  opt.alpha   = 0.02;
  opt.beta    = 0.7;
  opt.mu      = 1e-2;
  opt.epsilon = 1e-8;
  opt.slow    = false;
end
end

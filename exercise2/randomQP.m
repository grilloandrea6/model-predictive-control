%% Create a random quadratic program
function prob = randomQP(dim, seed)

% Number of optimization variables
prob.dim = dim;

% Set the seed of the random generator so we can get the same 'random'
% problem twice if we want
randn('state',seed);

% Create the problem
%  min  0.5 z'Hz + q'z s.t. Gx <= d
prob.G  = randn(5*dim,dim);
prob.d  = ones(5*dim,1);
prob.H  = randn(dim); prob.H = prob.H * prob.H';
prob.q  = randn(dim,1);

% We've chosen d = 1, so the point 0 is always in the interior of the
% constraints
prob.z0 = zeros(dim,1);

% Helper functions
prob.f       = @(z) 0.5*z'*prob.H*z + prob.q'*z;
prob.gradF   = @(z) prob.H*z + prob.q;
prob.phi     = @(z) sum(-log(prob.d-prob.G*z));
prob.gradPhi = @(z) prob.G'*diag(1./(prob.d-prob.G*z))*ones(length(prob.d),1);

end

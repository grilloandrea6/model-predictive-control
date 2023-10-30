%%%
% MPC-425, Exercise 2
%%%
function [iterationNumber] = ex2 (prob,opt,dim)


%opt.mu = myMu;

%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%

% Initialize
kappa = opt.kappa0;
z     = prob.z0;
zprev = z;




% Outer loop
stats.outerIterations = 0;
stats.innerIterations = 0;
tic
while kappa > opt.epsilon % Stop once the barrier term is small
  stats.outerIterations = stats.outerIterations + 1;
  
  % Inner loop (centering step)
  while 1
    stats.innerIterations = stats.innerIterations + 1;
    if any(prob.d-prob.G*z < 0), error('Current point z is not feasible'); end

    %vvvvvvvvvvvvvvvv YOUR CODE HERE vvvvvvvvvvvvvvvvvvv

    % Compute search direction 
    sumLeft = 0;
    sumRight = 0;

    for i = 1:10
        sumLeft = sumLeft + prob.G(i,:)' * prob.G(i,:) / (prob.d(i) - prob.G(i,:) * z)^2;

        sumRight = sumRight + prob.G(i,:)' / (prob.d(i) - prob.G(i,:) * z);

    end


    Dz = (prob.H + kappa *   sumLeft  )\(-prob.H * z - prob.q - kappa * sumRight );

    %^^^^^^^^^^^^^^^^ YOUR CODE HERE ^^^^^^^^^^^^^^^^^^^

    if dim == 2, plot([z(1);zprev(1)],[z(2);zprev(2)],'k.-','markersize',10); drawnow; end

    t = 1;

    % Reduce t until z+t*Dz is feasible
    % (Backtrack for feasibility)
    % Use opt.beta / opt.alpha as the backtracking parameters

    while t > opt.epsilon && any(prob.G*(z+t*Dz) > prob.d)
      t = opt.beta*t;
    end

    % Reduce t to minimize f(z+t*Dz)
    % Use opt.beta / opt.alpha as the backtracking parameters

    while t > opt.epsilon
      fup   = prob.f(z+t*Dz) + kappa * prob.phi(z+t*Dz); 
      df    = prob.gradF(z)  + kappa * prob.gradPhi(z);
      fdown = prob.f(z)      + kappa * prob.phi(z) + opt.alpha*t*df'*Dz;

      if fup < fdown, break; end
      t=opt.beta*t;
    end

    % Termination condition
    % We stop if we're no longer making any progress, either because the
    % step size t is very small, or the size of the gradient is very small
    if t < opt.epsilon || norm(Dz) < opt.epsilon, break; end
    
    % Slow down convergence so we can see what's going on
    if opt.slow, t = min([t t/(20*norm(Dz)) ]); end
    
    % Take step
    zprev = z;
    z = z + t*Dz;
  end

  if dim == 2, plot(z(1),z(2),'ro'); drawnow; end

  % Decrease barrier parameter
  kappa = kappa * opt.mu;
end   
   
stats.solveTime = toc;
iterationNumber = stats.innerIterations;

%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% THE BARRIER METHOD %%%%%%%%%%%%%%%%%%%


%%
% Solve the optimization problem and compare
tic
[zopt,fval,flag] = quadprog(prob.H,prob.q,prob.G,prob.d);
stats.quadprogSolveTime = toc;
if flag ~= 1, error('Could not solve optimization problem'); end

fprintf('---------------------\n');
fprintf('Your optimal cost: %f\n', prob.f(z));
fprintf('True optimal cost: %f\n', fval);
fprintf('Error = %f\n\n', abs(prob.f(z) - fval));


if dim==2
  fprintf('---------------------\n');
  fprintf('Your optimal solution: [%.3f %.3f]\n', z(1), z(2));
  fprintf('True optimal solution: [%.3f %.3f]\n', zopt(1), zopt(2));
  fprintf('Norm of error = %f\n\n', norm(z-zopt));
end

fprintf('---------------------\n');
fprintf('Solution statistics:\n');
fprintf('  Number of outer iterations: %i\n', stats.outerIterations);
fprintf('  Number of inner iterations: %i\n', stats.innerIterations);
fprintf('  Solve time:                 %.2f seconds\n', stats.solveTime);
fprintf('  Matlab solve time:          %.2f seconds\n', stats.quadprogSolveTime);

end






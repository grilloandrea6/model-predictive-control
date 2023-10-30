
%% Plot the optimization problem (if it is 2D)
function plotOptimization(prob)
if prob.dim ~= 2
  fprintf('Can only plot two dimensional optimization problems\n');
  return
end
clf; hold on; grid on;
try
  P = Polyhedron(prob.G,prob.d);
  P.plot('alpha',0.1,'linewidth',3,'color','k');
  %plotPolyLine(polytope(prob.G,prob.d),'k',3);
  
  %[X,Y] = gridPolytope(polytope(prob.G,prob.d),50);
  [B,l,u]=bounding_box(polytope(prob.G,prob.d));
  [X,Y] = meshgrid(linspace(l(1)-0.1,u(1)+0.1,50),linspace(l(2)-0.1,u(2)+0.1,50));
  F = 0*X;
  for i = 1:size(X,1)
    for j = 1:size(X,2)
      z = [X(i,j);Y(i,j)];
      F(i,j) = prob.f(z);
    end
  end
  contour(X,Y,F,20);
catch
  warning('Could not plot the constraints - possibly MPT is not installed?');
end

% Compute optimal solution and plot
try
  [zopt,fval,flag] = quadprog(prob.H,prob.q,prob.G,prob.d);
  if flag ~= 1, error('Could not solve optimization problem'); end
  plot(zopt(1),zopt(2),'ko','markersize',10,'markerfacecolor','k');
  text(zopt(1)+0.05,zopt(2),'Optimal point','fontweight','bold','fontsize',12,'backgroundcolor','w');
  plot(prob.z0(1),prob.z0(2),'ko','markersize',10,'markerfacecolor','k');
  text(prob.z0(1)+0.05,prob.z0(2),'Initial point','fontweight','bold','fontsize',12,'backgroundcolor','w');
catch
  warning('Could not compute the optimal solution - possibly optimization toolbox isn''t installed?');
end

% Compute analytic center and plot
try
  x = sdpvar(size(prob.G,2),1);
  dd = optimize(prob.d-prob.G*x >= 0, -geomean(prob.d-prob.G*x));
  if dd.problem ~= 0, error('Could not compute analytic center'); end
  x = double(x);
  plot(x(1),x(2),'ko','markersize',10,'markerfacecolor','k');
  text(x(1)+0.05,x(2),'Analytic center','fontweight','bold','fontsize',12,'backgroundcolor','w');
  axis tight
catch
  warning('Could not compute analytic center - possibly YALMIP isn''t installed?');
end

end


clear all
clc
rng default
lb=             [10 30 175 1];                     %Lower Bound of design variables
ub=             [30 60 225 3];                       %Upper Bound of desing variables
Pop_size=       60;                            %Size of the population
n_var=          4;                           %Number of variables involved in the objective function
intcon= [4];                

% options.InitialPopulationRange= [-400;400];

options=        optimoptions('gamultiobj','PopulationSize',Pop_size,'ParetoFraction',1,'MutationFcn',...
                            {@mutationadaptfeasible},'PopulationType','doubleVector','UseVectorized',true,'OutputFcn',@outputfunction);
%% 
options.CrossoverFraction=  0.95;
[x,fval,exitflag,output] =      gamultiobj(@fitness1,n_var,[],[],[],[],lb,ub,[],intcon,options);

% f1=             sort(fval(:,1),'ascend');
% f2=             sort(fval(:,2),'descend');
% plot(f1,f2,'-o');
% 

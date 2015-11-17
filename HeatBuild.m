%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                                                     %%%
%%%   File name:     HeatBuild.m                                        %%%               
%%%                                                                     %%%
%%%   Description:   Optimal design and control of heating system       %%%
%%%                                                                     %%%
%%%   Author:        Araz Ashouri - araz_ashouri@ieee.org               %%%
%%%   Created:       01 November 2015                                   %%%
%%%                                                                     %%%
%%%                  Copyright © 2015 Araz Ashouri                      %%%
%%%                                                                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Initialization

% Deleting the variables in the workspace
clear

% Clearing the command window
clc

% Modifying path to include the Yalmip toolbox
addpath(genpath('c:\yalmip'))

%% Parameters

disp('Initializing...')
tic; % Starting the chronometer

ts = 3600; % Sampling time [sec] (This should match the sampling time of data) 
C = 5e8; % Total heat capacity of building [J/K]
U = 600; % Total external heat transmittance of building [W/K]
SRE = 1000; % Energy Reference Surface [m2]
N = 8760; % Maximum time step allowed (length of the design problem)
Tdist = 273.15+35; % Distribution temperature for the heating system [K]
% Limits on the size of heat-pump
P_ahp_min = 3000; % Minimum dimension (nominal power) of air-source heat-pump (AHP) [W]
P_ahp_max = 10000; % Maximum dimension (nominal power) of AHP [W]
% Solver
options = sdpsettings('verbose',0,'solver','GLPK'); % GLPK is selected as the solver
options.glpk.itlim = 1e5; % Iteration limit for GLPK
% Temperature boundaries
Tint_min = zeros(N,1)+273.15+23; % Vector of minimum internal temperature [K]

%%% Hint: Play around with the values of C, U, and Tint_min, to see the effect on the final design.


%% Loading the measurement data
load('data.mat'); 
% elec_price: Two-level electricity price [CHF/kWh]
% Text: External Temperature for city of Geneva [K]
% Psol: Solar heat-gain [W]
% Pelec: Appliances heat-gain [W]
% Pven: Ventilation heat-gain [W/m2]

Pven = Pven * SRE; % [W/m2] --> [W]
Pint = Pelec + Pven; % Internal heat-gain [W]


%% Calculating the Coefficient of Performance

% This is just a simplified way of calculating COP for an AHP.
COP = 13.39*exp(-0.047*(Tdist-Text))+1.109*exp(0.012*(Tdist-Text));

%% Matrices for the temperature equation

% Continuous state-space matrices
Ac = -U/C;
Bc = [U/C 1/C];

nx = length(Ac);  % Number of states (= number of energy storage)

% Discrete state-space matrices (conversion using an exact discretization)
Ad = exp(ts*Ac);                 
Bd = inv(Ac)*(Ad - eye(nx))*Bc;  

disp(['Initialization is completed in ',num2str(toc),' seconds.']) % Stopping the chronometer
disp('********************************************')

%% Design

disp('Starting the design process...')

tic; % Starting the chronometer

% Clearing Yalmip variables
yalmip('clear')
% Manipulated design variables
Tint = sdpvar(N+1,1,'full'); % Internal temperature [K]
Pheat = sdpvar(N,1,'full'); % Output power of the heating system [W]
cost_elec = sdpvar(N,1,'full'); % Cost of the electricity [CHF]
cost_ahp = sdpvar(1,1,'full'); % Cost of the AHP [CHF]
P_ahp = sdpvar(1,1,'full'); % Dimension (nominal power) of the AHP [W]
P_ahp_in = sdpvar(N,1,'full'); % Inlet electric power of the AHP [W]
constraints = []; % Initializing the vector of constraints

%% Optimization

% Constraint on the initial internal temperature
constraints = [constraints, Tint(1) == Tint_min(1) + 0.5];

% Internal temperature equation: This represents a 1R1C thermal model.
% Refer to the thesis cited in the README file.
constraints = [constraints, Tint(2:N+1) == Ad*Tint(1:N)+Bd(1)*Text(1:N)+Bd(2)*(Pint(1:N)+Psol(1:N)+Pheat(1:N))];
    
% Constraint on the internal temperature
constraints = [constraints, Tint(1:N) >= Tint_min(1:N)]; 

% Constraint on the ahp powers
constraints = [constraints, P_ahp >= P_ahp_min]; % AHP size is limited
constraints = [constraints, P_ahp <= P_ahp_max];
constraints = [constraints, P_ahp_in(1:N) >= 0];
constraints = [constraints, P_ahp_in(1:N) <= P_ahp]; % Input power is limited by size

% Constraint on the heating system
constraints = [constraints, Pheat(1:N) >= 0];
constraints = [constraints, Pheat(1:N) == COP(1:N).*P_ahp_in(1:N)]; 

% Electricity cost
constraints = [constraints, cost_elec(1:N) >= 0]; 
constraints = [constraints, cost_elec(1:N) == elec_price(1:N).*P_ahp_in(1:N)/1e3]; 

% AHP cost (investment and mintenance cost)
constraints = [constraints, cost_ahp >= 0]; 
constraints = [constraints, cost_ahp == (2.2*P_ahp - 4700)]; 

% Optimization objective (scalar value)
objective = cost_ahp + sum(cost_elec);

% Calling the solver for optimization and retrieving the results
diagnostics = solvesdp(constraints,objective,options);

disp(['Design is completed in ',num2str(toc),' seconds.']) % Stopping the chronometer
disp('********************************************')
disp(['Optimization result: ',diagnostics.info])
disp('********************************************')

  
%% Retrieving results

% Converting the results (Yalmip variables) into real numbers
P_ahp_opt = double(P_ahp);
disp(['Optimal dimension of AHP is ',num2str(floor(P_ahp_opt)),' [W]'])
Tint_opt = double(Tint); % Optimal internal temperaure [K]
Pheat_opt = double(Pheat);  % Optimal output power of the heating system [W]
cost_ahp_opt = double(cost_ahp); % Optimal cost of the heat-pump [CHF]
disp(['The cost of AHP is ',num2str(floor(cost_ahp_opt)),' [CHF]'])
cost_elec_opt = double(cost_elec); % Optimal cost of the electricity [CHF]
disp(['The anuual operating cost is ',num2str(floor(sum(cost_elec_opt))),' [CHF]'])
total_cost = double(objective);
disp(['The total cost is ',num2str(floor(total_cost)),' [CHF]'])

% End of file

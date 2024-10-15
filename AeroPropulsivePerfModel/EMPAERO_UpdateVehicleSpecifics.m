function [Vehicle] = EMPAERO_UpdateVehicleSpecifics(Vehicle)
load empModel.mat

Vehicle.Aero.DesSurf = [];                                                  % set desired lifting surface to analyse to none : all surfaces evalutated
% choose control surface type
Vehicle.Aero.ControlSurfaceType = 'SingleSlotted';                          % single-slotted control surface
% Vehicle.Aero.ControlSurfaceType = 'Plain';                                % plain control surface

%% Extract 2D data for each lifting surfaces 
WINGAERO = Vehicle.Aero.WingAero;
HTAERO = Vehicle.Aero.HTAero;
VTAERO = Vehicle.Aero.VTAero;

%% Compute lift curve slope based on 2D data 
% lift curve slope per deg
AOAq = [-4 6];
% Wing
Cl_Wing = interp1(WINGAERO(:,1),WINGAERO(:,2),AOAq);
Vehicle.Aero.WingCla = diff(Cl_Wing)/diff(AOAq);
% Horizontal Stabilizer 
Cl_HTail = interp1(HTAERO(:,1),HTAERO(:,2),AOAq);
Vehicle.Aero.HTailCla = diff(Cl_HTail)/diff(AOAq);
% Vertical Stabilizer
Cl_VTail = interp1(VTAERO(:,1),VTAERO(:,2),AOAq);
Vehicle.Aero.VTailCla = diff(Cl_VTail)/diff(AOAq);

%% Update Cla for each lifting surface 

Vehicle.Aero.RWing.Cla = Vehicle.Aero.WingCla;
Vehicle.Aero.RHTail.Cla = Vehicle.Aero.HTailCla;
Vehicle.Aero.VTail.Cla = Vehicle.Aero.VTailCla;
%% Compute zero-lift anlge of attack
% Based on DATCOM (Reader pg. 496) for untwisted, constant section wings.
% Applicable to swept wings if the airfoils are defined parallel to the
% free stream

% Wing
cl_i = 0.0; % section design lift coefficient
alpha_i = interp1(WINGAERO(:,2),WINGAERO(:,1),cl_i,"linear"); % anlge of attack for design lift coefficient
Wing_zero_lift_AOA = alpha_i - cl_i/Vehicle.Aero.WingCla;

% Horizontal tail
cl_i = 0.0; % section design lift coefficient
alpha_i = interp1(HTAERO(:,2),HTAERO(:,1),cl_i,"linear"); % anlge of attack for design lift coefficient
HT_zero_lift_AOA = alpha_i - cl_i/Vehicle.Aero.HTailCla;

% Vertical stabilizer
cl_i = 0.0; % section design lift coefficient
alpha_i = interp1(VTAERO(:,2),VTAERO(:,1),cl_i,"linear"); % anlge of attack for design lift coefficient
VT_zero_lift_AOA = alpha_i - cl_i/Vehicle.Aero.VTailCla;

%% Definition and Initialization ( 04.02.2023, kunwar)
% Airfoil zero-lift line definition
Vehicle.Aero.RWing.zero_lift_AOA = Wing_zero_lift_AOA;                      % degree
Vehicle.Aero.RHTail.zero_lift_AOA = HT_zero_lift_AOA;                        % degree
Vehicle.Aero.VTail.zero_lift_AOA = VT_zero_lift_AOA;                          % degree

% Airfoil max-lift anlge of attack (taken for airfoil)
Vehicle.Aero.RWing.max_lift_AOA = WINGAERO(WINGAERO(:,2) == max(WINGAERO(:,2)),1);                                     % degree
Vehicle.Aero.RHTail.max_lift_AOA = HTAERO(HTAERO(:,2) == max(HTAERO(:,2)),1);                                      % degree
Vehicle.Aero.VTail.max_lift_AOA = VTAERO(VTAERO(:,2) == max(VTAERO(:,2)),1);                                     % degree

% Downwash angle initialization
Vehicle.Aero.RWing.epsilon = 0;                                             % degree
Vehicle.Aero.RHTail.epsilon = 0;                                              % degree
Vehicle.Aero.VTail.epsilon = 0;                                               % degree

% update for geometric symetric halfs 
Vehicle.Aero.LWing = Vehicle.Aero.RWing;
Vehicle.Aero.LHTail = Vehicle.Aero.RHTail;
Vehicle.Aero.VTail = Vehicle.Aero.VTail;
end
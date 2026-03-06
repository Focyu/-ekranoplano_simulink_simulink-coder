clear all 
close all
clc
%% Initialization
Cp = 0.248;% motor efficientcy factor
Tp = 0.125e-03; % motor PWM switching frequency
max_thrust_force_per_motor =  Tp;

% Test 1
tsim = 10;
step = 0.01;
x_nom = zeros(12,1);
% 1. Velocidad inicial de crucero (basada en vc del Model.m)
x_nom(1) = 27;     % 'u' = 28 m/s (Velocidad de sustentación adecuada)
x_nom(3) = 0;      % 'w' = 0 (Sin velocidad vertical inicial)

% 2. Ángulo de ataque de equilibrio (Pitch trim inicial)
x_nom(8) = 1.0 * (pi/180);  % Theta (Pitch) para generar Lift

% 3. Posición inicial
x_nom(10) = 0;     % x_NED
x_nom(11) = 0;     % y_NED
x_nom(12) = -1.80;  % altura inicial (invertida)

u_nom = zeros(5,1);
u_nom(2) = -1.5 * pi/180; % Ligero elevador hacia arriba para compensar el Pitch-down del IGE
u_nom(4:5) = Tp * 0.6;      % Acelerador al 60% inicial (No al máximo)

x0 = x_nom;
% file_name = 'test_1.csv';
% [XDOT] = Model(x_nom,u_nom) % debug
%% PARAMETROS DE CONTROL PID (Lazo Cerrado)

% 1. Lazo de Velocidad (Throttle / Motores) - NUEVO
% Su único objetivo es vencer el drag y mantener la presión dinámica.
u_sp = 28.0; % Setpoint: Velocidad de crucero deseada en m/s
% Ganancias sugeridas: Los motores tienen inercia, mejor usar PI dominante.
Kp_u = 0.05;    
Ki_u = 0.01;  
Kd_u = 0.005;   

% 2. Lazo de Altura (Controla el Setpoint de Cabeceo) - REDISEÑADO
% En lugar de mover el motor, dicta qué ángulo de nariz se necesita.
h_sp = 1.0; % Setpoint: 1 metro de altura sobre el agua
% Este PID calculará un theta_sp (ángulo deseado). Ganancias suaves para no cabecear bruscamente:
Kp_h = 2.5;  % Si caigo 1m, pido 2.5 rad extra de pitch (luego se satura a valores seguros en Simulink)
Ki_h = 0.1;
Kd_h = 1.0;

% Límite de seguridad para el setpoint de Pitch (para aplicar dentro de Simulink)
theta_max =  8.0 * (pi/180); % Máximo cabeceo permitido hacia arriba
theta_min = -2.0 * (pi/180); % Máximo cabeceo permitido hacia abajo

% 3. Lazo Interno de Elevador (Control de Cabeceo / Pitch)
% Sigue al theta_sp dictado por el lazo de altura.
% Ganancias iniciales (negativas por convención de deflexión):
Kp_pitch = -25.0;   
Ki_pitch = -5.0;  
Kd_pitch = -12.5;   

% 4. Lazo de Timón (Control de Dirección / Yaw)
psi_sp = 0 * (pi/180); % Setpoint
Kp_yaw = -15.0;   
Ki_yaw = -0.5;  
Kd_yaw = -5.0;   

% 5. Lazo de Alerones (Estabilización de Alabeo / Roll)
phi_sp = 0.0; % Alas horizontales
Kp_roll = -15.0;   
Ki_roll = -1.0;  
Kd_roll = -10.0;   

%% Simulation
sim = sim('pid_control_V1.slx');

S = sim.states;
t = sim.tout;
% C = sim.control.*ones(length(t),5); %antes se transformaba
C = sim.control_out; %sim.control_out será directamente una matriz de dimensión

LD_ratio = sim.LD;
Fbx = sim.Fbx;
Fby = sim.Fby;
Fbz = sim.Fbz;
Mx = sim.Mbx;
My = sim.Mby;
Mz = sim.Mbz;

CQ = sim.CQ;
Cl = sim.Cl;
Cn = sim.Cn;
Cm = sim.Cm;
alpha = sim.alpha;
beta = sim.beta;

CL_w_OGE = sim.CL_w_OGE;
CL_h_OGE = sim.CL_h_OGE;
CL_w_IGE = sim.CL_w_IGE;
CL_h_IGE = sim.CL_h_IGE;
CD_iw_IGE = sim.CD_iw_IGE;
CD_ih_IGE = sim.CD_ih_IGE;

Fgx = sim.Fgx;
Fgy = sim.Fgy;
Fgz = sim.Fgz;
Fax = sim.Fax;
Fay = sim.Fay;
Faz = sim.Faz;
Ftx = sim.Ftx;
Fty = sim.Fty;
Ftz = sim.Ftz;

P = sim.Power;
E = sim.Energy;
LT = sim.Load_Torque;

% Creating the .csv file for numerical export of results
% T1 = table(LD_ratio,Fbx,Fby,Fbz,Mx,My,Mz,CQ,Cl,Cm,Cn,alpha,beta,CL_w_OGE,CL_h_OGE,CL_w_IGE,CL_h_IGE,CD_iw_IGE,CD_ih_IGE,'VariableNames',["LD_rat","Fbx","Fby","Fbz","Mx","My","Mz","CQ","Cl","Cm","Cn","Alpha","Beta","CL_w_OGE","CL_h_OGE","CL_w_IGE","CL_h_IGE","CD_iw_IGE","CD_ih_IGE"]);
% T2 = table(S(:,1),S(:,2),S(:,3),S(:,4),S(:,5),S(:,6),S(:,7),S(:,8),S(:,9),S(:,10),S(:,11),S(:,12),'VariableNames',["u","v","w","p","q","r","phi","theta","psi","x_NED","y_NED","z_NED"]);
% T3 = table(C(:,1),C(:,2),C(:,3),C(:,4),C(:,5),'VariableNames',["aileron","elevator","rudder","thrust_1","thrust_2"]);
% writetable([T1 T2 T3],file_name);
% 
% %% Plotting
% % LD ratio
% figure
% plot(t,LD_ratio(:,1))
% xlabel('t [s]')
% ylabel('L/D')
% title('LD ratio')
% grid on
% %-------------------Alpha and beta----------
% figure
% subplot(2,1,1)
% plot(t,alpha(:,1))
% xlabel('t [s]')
% ylabel('\alpha [rad]')
% title('Angle of attack')
% grid on
% 
% subplot(2,1,2)
% plot(t,beta(:,1))
% xlabel('t [s]')
% ylabel('\beta [rad]')
% title('Side-slip Angle')
% grid on
% %-------------------Power, Energy & Load Torque-------------------
% figure
% subplot(3,1,1)
% plot(t,P(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('P [kW]')
% grid on
% title('Power')
% 
% subplot(3,1,2)
% plot(t,E(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('E [kWh]')
% grid on
% title('Energy')
% 
% subplot(3,1,3)
% plot(t,LT(:,1),'LineWidth',1.2)
% xlabel('t[s]')
% ylabel('T_L [Nm]')
% grid on
% title('Load Torque')
% % ----------------forces and torques---------------------------------------------
% figure
% subplot(2,2,1)
% hold on
% plot(t,Fbx(:,1),'LineWidth',1.2)
% plot(t,Fby(:,1),'LineWidth',1.2)
% plot(t,Fbz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_b [N]')
% title('F_b')
% legend('Fbx','Fby','Fbz')
% grid on
% 
% subplot(2,2,2)
% hold on
% plot(t,Fax(:,1),'LineWidth',1.2)
% plot(t,Fay(:,1),'LineWidth',1.2)
% plot(t,Faz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_a [N]')
% title('F_a')
% legend('Fax','Fay','Faz')
% grid on
% 
% subplot(2,2,3)
% hold on
% plot(t,Ftx(:,1),'LineWidth',1.2)
% plot(t,Fty(:,1),'LineWidth',1.2)
% plot(t,Ftz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_t [N]')
% title('F_t')
% legend('Ftx','Fty','Ftz')
% grid on
% 
% subplot(2,2,4)
% hold on
% plot(t,Fgx(:,1),'LineWidth',1.2)
% plot(t,Fgy(:,1),'LineWidth',1.2)
% plot(t,Fgz(:,1),'LineWidth',1.2)
% hold off
% xlabel('t [s]')
% ylabel('F_g [N]')
% title('F_g')
% legend('Fgx','Fgy','Fgz')
% grid on
% 
% 
% figure
% subplot(3,1,1)
% plot(t,Mx(:,1))
% xlabel('t [s]')
% ylabel('M_b_x [N]')
% title('M_b_x')
% grid on
% 
% subplot(3,1,2)
% plot(t,My(:,1))
% xlabel('t [s]')
% ylabel('M_b_y [N]')
% title('M_b_y')
% grid on
% 
% subplot(3,1,3)
% plot(t,Mz(:,1))
% xlabel('t [s]')
% ylabel('M_b_z [N]')
% title('M_b_z')
% grid on
% sgtitle('Torque vector M_b')
% % ----------------Aerodyn Coeff---------------------------------------------
% CL_w_OGE = sim.CL_w_OGE;
% CL_h_OGE = sim.CL_h_OGE;
% CL_w_IGE = sim.CL_w_IGE;
% CL_h_IGE = sim.CL_h_IGE;
% CD_iw_IGE = sim.CD_iw_IGE;
% CD_ih_IGE = sim.CD_ih_IGE;
% 
% figure
% subplot(1,2,1)
% plot(t,CL_w_OGE(:,1))
% xlabel('t [s]')
% ylabel('CL_w_{OGE}')
% title('CL_w_{OGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CL_h_OGE(:,1))
% xlabel('t [s]')
% ylabel('CL_h_{OGE}')
% title('CL_h_{OGE}')
% grid on
% sgtitle('Aerodyn Coefficient from lift forces OGE')
% 
% figure
% subplot(1,2,1)
% plot(t,CL_w_IGE(:,1))
% xlabel('t [s]')
% ylabel('CL_w_{IGE}')
% title('CL_w_{IGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CL_h_IGE(:,1))
% xlabel('t [s]')
% ylabel('CL_h_{IGE}')
% title('CL_h_{IGE}')
% grid on
% sgtitle('Aerodyn Coefficient from lift forces IGE')
% 
% figure
% subplot(1,2,1)
% plot(t,CD_iw_IGE(:,1))
% xlabel('t [s]')
% ylabel('CD_{iw}_{IGE}')
% title('CD_{iw}_{IGE}')
% grid on
% 
% subplot(1,2,2)
% plot(t,CD_ih_IGE(:,1))
% xlabel('t [s]')
% ylabel('CD_{ih}_{IGE}')
% title('CD_{ih}_{IGE}')
% grid on
% sgtitle('Aerodyn Coefficient from drag forces IGE')
% %--------------------------------------------------------
% figure
% plot(t,CQ(:,1))
% xlabel('t [s]')
% ylabel('C_Q')
% title('C_Q')
% grid on
% title('Aerodyn Coefficient from side forces')
% 
% figure
% subplot(3,1,1)
% plot(t,Cl(:,1))
% xlabel('t [s]')
% ylabel('C_l')
% title('C_l')
% grid on
% 
% subplot(3,1,2)
% plot(t,Cm(:,1))
% xlabel('t [s]')
% ylabel('C_m')
% title('C_m')
% grid on
% 
% subplot(3,1,3)
% plot(t,Cn(:,1))
% xlabel('t [s]')
% ylabel('C_n')
% title('C_n')
% grid on
% sgtitle('Aerodyn Coeff from torques')
% 
% % ----------------States---------------------------------------------
% % Linear velocities
% figure
% plot(t,S(:,1),t,S(:,2),t,S(:,3))
% xlabel('t [s]')
% ylabel('[m/s]')
% title('Linear velocities')
% grid on
% legend('u','v','w')
% %-----------------------------angular velocities
% figure
% plot(t,S(:,4),t,S(:,5),t,S(:,6))
% xlabel('t [s]')
% ylabel('[rad/s]')
% title('Angular velocities')
% grid on
% legend('p','q','r')
% 
% %-----------------------Euler angles
% figure
% plot(t,S(:,7),t,S(:,8),t,S(:,9))
% xlabel('t [s]')
% ylabel('[rad]')
% title('Euler angles')
% grid on
% legend('\phi','\theta','\psi')
% 
% figure
% hold all
% plot(t,-S(:,12))
% xlabel('t[s]')
% ylabel('h_{NED}[m]')
% title('Height in NED')
% grid on
% 
% 
% figure
% hold all
% plot(t,S(:,10))
% plot(t,S(:,11))
% plot(t,S(:,12))
% xlabel('t[s]')
% ylabel('[m]')
% title('Positions in NED')
% legend('x_{NED}','y_{NED}','z_{NED}')
% grid on
% 
% figure
% hold all
% plot3(S(1,10),S(1,11),-S(1,12),'rx')
% plot3(S(:,10),S(:,11),-S(:,12),'b')
% xlabel('x_{NED}')
% ylabel('y_{NED}')
% zlabel('h_{NED}')
% title('Trajectory in 3D')
% grid on
% legend('(x_0,y_0,h_0)','(x(t),y(t),h(t))')
% %--------------other-----------------
% gama = S(:,8)-atan2(S(:,3),S(:,1));
% figure
% plot(t,gama)
% xlabel('t [s]')
% ylabel('\gamma [rad]')
% title('Flight Path Angle \gamma')
% grid on
% 
% Va = sqrt(S(:,1).^2+S(:,2).^2+S(:,3).^2);
% figure
% plot(t,Va)
% xlabel('t [s]')
% ylabel('V_a [m/s]')
% title('True Airspeed V_a')
% grid on
% % ----------------Control---------------------------------------------
% figure
% subplot(5,1,1)
% plot(t,C(:,1))
% xlabel('t [s]')
% ylabel('\delta_a [deg]')
% title('Aileron Deflection')
% grid on
% 
% subplot(5,1,2)
% plot(t,C(:,2))
% xlabel('t [s]')
% ylabel('\delta_e [deg]')
% title('Elevator Deflection')
% grid on
% 
% subplot(5,1,3)
% plot(t,C(:,3))
% xlabel('t [s]')
% ylabel('\delta_r [deg]')
% title('Rudder Deflection')
% grid on
% 
% subplot(5,1,4)
% plot(t,C(:,4).*100./max_thrust_force_per_motor)
% xlabel('t [s]')
% ylabel('\delta_{t1} [%]')
% title('Throttle 1')
% grid on
% 
% subplot(5,1,5)
% plot(t,C(:,5).*100./max_thrust_force_per_motor)
% xlabel('t [s]')
% ylabel('\delta_{t2} [%]')
% title('Throttle 2')
% grid on

% %% Save all figures automatically
% 
% output_folder = '/home/facyu/Descargas';
% 
% % Crear la carpeta si no existe
% if ~exist(output_folder, 'dir')
%     mkdir(output_folder);
% end
% 
% % Obtener todas las figuras abiertas
% figs = findall(0,'Type','figure');
% 
% for i = 1:length(figs)
%     filename = fullfile(output_folder, sprintf('Figura_%02d.png', i));
%     exportgraphics(figs(i), filename, 'Resolution', 300);
% end
% 
% disp('Todas las figuras fueron guardadas correctamente.')
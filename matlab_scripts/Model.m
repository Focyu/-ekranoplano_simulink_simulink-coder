function [XDOT] = Model(X,U)

%-----------STATE AND CONTROL VECTORS-------------
x1 = X(1); %u
x2 = X(2); %v
x3 = X(3); %w
x4 = X(4); %p
x5 = X(5); %q
x6 = X(6); %r
x7 = X(7); %phi
x8 = X(8); %theta
x9 = X(9); %psi
x10 = X(10); % x_NED
x11 = X(11); % y_NED
x12 = X(12); % z_NED

u1 = U(1); %d_A (Aileron)
u2 = U(2); %d_E (Elevator)
u3 = U(3); %d_R (Rudder)
u4 = U(4); %d_th1 (throttle 1)
u5 = U(5); %d_th2 (throttle 2)

%-----------SYSTEM PARAMETERS------------------------------
m = 112; % Aircraft mass (Kg) - ASEGURAR QUE COINCIDA CON GAZEBO SDF/URDF
vc = 28; % cruise speed
bw = 5.02; % Span (Wing)
bh = 2.74; % Span (HTP)
cbar = 0.646; % Mean Aerodynamic Chord (m)
lt = 24.8;  % Distance of AC of tail and body (m)
Sw = 3.334;    % Wing planform area (m^2)
Sh = 1.128;    % Tail planform area (m^2)
ARw = 7.5; % Apect Ratio (Wing) 
ARh = 6.7; % Apect Ratio (HTP) 
TRw = 0.4; % Taper Ratio (Wing) 
TRh = 0.5; % Taper Ratio (HTP) 

Xcg = 0;  % x position of CG in Fm (m)
Ycg = 0;  % y position of CG in Fm (m)
Zcg = 0;  % z position of CG in Fm (m)

Xac = 0.097;  % x position of Aerodynamic Center in Fm (m)
Yac = 0;      % y position of Aerodynamic Center in Fm (m)
Zac = -0.06;  % z position of Aerodynamic Center in Fm (m)

%Engine constants
Xapt1 = -0.519; % x position of engine 1 force in Fm (m)
Yapt1 = 0.6;    % y position of engine 1 force in Fm (m)
Zapt1 = 0.285;  % z position of engine 1 force in Fm (m)

Xapt2 = -0.519;  % x position of engine 2 force in Fm (m)
Yapt2 = -0.6;    % y position of engine 2 force in Fm (m)
Zapt2 = 0.285;   % z position of engine 2 force in Fm (m)

%Other constants
rho = 1.225;
g = 9.81;
e0w = 0.9; % Oswald's efficiency Factor (Wing)
e0h = 0.9; % Oswald's efficiency Factor (HTP)
alpha_0w = -3.75*pi/180; % Zero Lift Angle of Attack (Wing)
alpha_0h = -4.25*pi/180; % Zero Lift Angle of Attack (HTP)
iw = 1.0 * pi/180; % Angle of incidence (Wing)
ih = 0.5 * pi/180; % Angle of Incidence (HTP)
eps = 0; % Downwash Angle
zw = 0.363; % Wing Offset
zh = 2.50;  % HTP Offset (CORREGIDO: Altura realista de cola en T para AirFish 8)
CD_0 = 0.0306; % Zero Lift Drag Coefficient

%-----------CONTROL LIMITS SATURATION------------------------------

% aileron
u1min = -20*pi/180;
u1max =  20*pi/180;
% elevator
u2min = -20*pi/180;
u2max =  20*pi/180;
% rudder
u3min = -15*pi/180;
u3max =  15*pi/180;

% engines (0 a 1 normalizado)
u4min =  0;
u4max =  1.0;
u5min =  0;
u5max =  1.0;

if(u1>u1max)
    u1=u1max;
elseif(u1<u1min)
    u1=u1min;
end

if(u2>u2max)
    u2=u2max;
elseif(u2<u2min)
    u2=u2min;
end

if(u3>u3max)
    u3=u3max;
elseif(u3<u3min)
    u3=u3min;
end

if(u4>u4max)
    u4=u4max;
elseif(u4<u4min)
    u4=u4min;
end

if(u5>u5max)
    u5=u5max;
elseif(u5<u5min)
    u5=u5min;
end

%-------------INTERMEDIATE VARIABLES-----------------------------

%% When Va = Vg + Vw
u = x1; 
v = x2; 
w = x3;
phi = x7; 
theta = x8; 
psi = x9;
c_phi = cos(phi);
s_phi = sin(phi);
c_theta = cos(theta);
s_theta = sin(theta);
c_psi = cos(psi);
s_psi = sin(psi);

% rotation matrix
Rb_v = [c_theta*c_psi c_theta*s_psi -s_theta;
        s_phi*s_theta*c_psi-c_phi*s_psi s_phi*s_theta*s_psi+c_phi*c_psi s_phi*c_theta;
        c_phi*s_theta*c_psi+s_phi*s_psi c_phi*s_theta*s_psi-s_phi*c_psi c_phi*c_theta];

Vb_g = [u;v;w];
% steady-state component of wind
w_ns = 0;
w_es = 0;
w_ds = 0;
% gust component of wind
u_wg = 0;
v_wg = 0;
w_wg = 0;
Vb_w = Rb_v*[w_ns;w_es;w_ds]+[u_wg;v_wg;w_wg];

Va_b = [Vb_g(1)-Vb_w(1);Vb_g(2)-Vb_w(2);Vb_g(3)-Vb_w(3)];

u_r = Va_b(1);
v_r = Va_b(2);
w_r = Va_b(3);

Va = sqrt(u_r^2+v_r^2+w_r^2);
alpha = atan2(w_r,u_r);
beta = asin(v_r/Va);

h = -x12;
hw = max(0.001, h - zw); % Límite de seguridad para IGE
hh = max(0.001, h + zh); % Límite de seguridad para IGE

Q = 0.5*rho*Va^2;     %dynamic pressure

wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%--------------Aerodynamic Force Coefficients---------------------

% Lift
mu_Lw = 1 + (1 - 2.25 * (TRw^0.00273 - 0.997)*((ARw^0.717)+13.6))*(288 * abs(hw/bw)^0.787 * exp(-9.14*(abs(hw/bw)^0.327)))/(ARw^0.882);
mu_Lh = 1 + (1 - 2.25 * (TRh^0.00273 - 0.997)*((ARh^0.717)+13.6))*(288 * abs(hh/bh)^0.787 * exp(-9.14*(abs(hh/bh)^0.327)))/(ARh^0.882);

% Derivada del downwash (Aproximación lineal para alas rectas)
deps_dalpha = 0.35; 

% El downwash dinámico incluye el retraso de fase (lt/Va * q)
eps = deps_dalpha * (alpha - alpha_0w) + deps_dalpha * (lt/Va) * x5;

CL_w_OGE = 2*pi*(ARw/(ARw+2))*(alpha-alpha_0w+iw);

% Restamos el downwash dinámico (eps) al ángulo de ataque de la cola
CL_h_OGE = 2*pi*(ARh/(ARh+2))*(alpha-alpha_0h+ih-eps);

CL_w_IGE = CL_w_OGE*mu_Lw;
CL_h_IGE = CL_h_OGE*mu_Lh; % CORREGIDO: Removido C_L_e para usar puramente torques en el control de pitch

Lh = 0.5*rho*Va^2*CL_h_IGE*Sh;
Lw = 0.5*rho*Va^2*CL_w_IGE*Sw;
Ltot = Lw + Lh;

% Drag
mu_Dw = 1 - (1 - 0.157*max(0, (TRw^0.757 - 0.373))*max(0,(ARw^0.417 - 1.27)))*exp(-4.74*max(0,abs(hw/bw)^0.814))-abs(hw/bw)^2*exp(-3.88*max(0,abs(hw/bw)^0.758));
mu_Dh = 1 - (1 - 0.157*max(0, (TRh^0.757 - 0.373))*max(0,(ARh^0.417 - 1.27)))*exp(-4.74*max(0,abs(hh/bh)^0.814))-abs(hh/bh)^2*exp(-3.88*max(0,abs(hh/bh)^0.758));

C_D_e = -0.0000108*u2^2+0.000715*u2;

CD_iw_IGE = CL_w_IGE^2*mu_Dw/(pi*e0w*ARw);
CD_ih_IGE = CL_h_IGE^2*mu_Dh/(pi*e0h*ARh);

Dtot = 0.5*rho*Va^2*(CD_0*Sw + CD_iw_IGE*Sw + CD_ih_IGE*Sh + C_D_e*Sh);

% Side forces
CQ = -0.019*beta*180/pi; % steady-state
LD_ratio = Ltot/Dtot;

%------------------------Dimensional Aero Forces--------------------

% Actual dimensional forces in F_s (Stability axis)
FA_s = [-Dtot;
         CQ*Q*Sw;
        -Ltot];

% Rotate forces to F_b (body axis)
C_bs = [cos(alpha)  0  -sin(alpha);
        0           1   0;
        sin(alpha)  0   cos(alpha)];

% Sequential Rotations
FA_b = C_bs*FA_s;

%------------------ Cálculo de Torques Aerodinámicos ------------------
% Parámetros de estabilidad longitudinal (usando Radianes para la dinámica pura)
Cm_alpha = -1.14; % Equivalente aproximado a -0.02 * 180/pi
Cm_q = -5.0;      % Amortiguación del cabeceo 
Cm_de = -3.0;     % Autoridad del elevador 

% Factor de Momento por Efecto Suelo (Pitch-down moment debido al AC shift)
Cm_h = 0.05;      % Constante empírica del desplazamiento del AC
Delta_Cm_IGE = -Cm_h * exp(-4.0 * abs(hw/bw)); % Momento inducido por el suelo

Cl = -0.0005*beta*180/pi; % Eje de alabeo (Roll)

% Nuevo Cm: Momento por alfa + Momento por IGE + Momento de velocidad de cabeceo (q) + Elevador
Cm = Cm_alpha*alpha + Delta_Cm_IGE + Cm_q*(x5*cbar/(2*Va)) + Cm_de*u2; 

Cn = -0.002*beta*180/pi; % Eje de guiñada (Yaw)

CMac_b = [Cl;Cm;Cn];% steady-state
%-------------------------Aero moment about CG--------------------------
MAcg_b = CMac_b.*[bw*Q*Sw;cbar*Q*Sw;bw*Q*Sw];

%-------------------------Engine Force & Moment-------------------------
%propeller thrust - Fitzpatrick model for ACP 25x12.5E
km = 39.51;% motor constant [m/s] torque/sqrt(power) 
Cp = 0.57;% efficientcy factor
D = 0.80; % diameter [m]
Sp = pi*(D^2/4); % disc area

d_t1 = u4; % CORREGIDO: range [0,1] puro, sin dividir por Tp
Vd1 = Va + d_t1*(km-Va);
Tp1 = 0.5*rho*Sp*Cp*Vd1*(Vd1-Va);

d_t2 = u5; % CORREGIDO: range [0,1] puro, sin dividir por Tp
Vd2 = Va + d_t2*(km-Va);
Tp2 = 0.5*rho*Sp*Cp*Vd2*(Vd2-Va);

FE1_b = [Tp1*cos(-5*pi/180);
        0;
        sin(5*pi/180)];
FE2_b = [Tp2*cos(-5*pi/180);
        0;
        sin(5*pi/180)];
FE_b = FE1_b + FE2_b;

%Now engine moment due to offset of thrust from CG
mew1 = [Xcg-Xapt1;
        Yapt1-Ycg;
        Zcg-Zapt1];

mew2 = [Xcg-Xapt2;
        Yapt2-Ycg;
        Zcg-Zapt2];

MEcg1_b = cross(mew1,FE1_b);
MEcg2_b = cross(mew2,FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;
%----------------------------Gravity Effect-----------------------------
%Calculate gravitational forces in body frame. This causes no moment about cg
g_b = [-g*sin(x8);g*cos(x8)*sin(x7);g*cos(x8)*cos(x7)];
Fg_b = m*g_b;

%----------------------------State Derivatives------------------------
% Fuerzas traslacionales
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);

% Dinámica Rotacional (Ecuaciones de Euler con producto de inercia Ixz)
Ixx = 39.71;
Iyy = 85.51;
Izz = 114.39;
Ixz = 8.97;

Gamma = Ixx*Izz - Ixz^2; 

p = wbe_b(1);
q = wbe_b(2);
r = wbe_b(3);

Mcg_b = MAcg_b + MEcg_b;
L = Mcg_b(1);
M = Mcg_b(2);
N = Mcg_b(3);

p_dot = (Izz*L + Ixz*N - (Ixz*(Iyy - Ixx - Izz)*p + (Ixz^2 + Izz*(Izz - Iyy))*r)*q) / Gamma;
q_dot = (M - (Ixx - Izz)*p*r - Ixz*(p^2 - r^2)) / Iyy;
r_dot = (Ixz*L + Ixx*N + (Ixz*(Iyy - Ixx - Izz)*r + (Ixz^2 + Ixx*(Ixx - Iyy))*p)*q) / Gamma;

x4to6dot = [p_dot; q_dot; r_dot];

H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9dot = H_phi*wbe_b;
x10to12dot = rot_body_to_ned(X(1:9));
XDOT = [x1to3dot;
    x4to6dot;
    x7to9dot;
    x10to12dot;
    LD_ratio;
    F_b;
    Mcg_b;
    CQ;
    Cl;
    Cm;
    Cn;
    alpha;
    beta;
    CL_w_OGE;
    CL_h_OGE;
    CL_w_IGE;
    CL_h_IGE;
    CD_iw_IGE;
    CD_ih_IGE;
    Fg_b;
    FE_b; 
    FA_b];

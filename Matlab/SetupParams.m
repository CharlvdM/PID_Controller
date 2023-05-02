%% Induction motor parameters
Vn  = 380;              % rms line-to-line, rated voltage [V]
Vp  = Vn/sqrt(3);       % rms phase rated voltage [V]
fn  = 50;               % rated frequency [Hz]
omega_n = fn * 2*pi;    % rated frequency [rad/s]
K = Vp/omega_n;         % to keep constant Vs/f. [V*s/rad]

Jr = 0.00518; % rotor moment of inertia [kg*m^2]
J = Jr*2;     % moment of inertia of the rotor and the load (another motor) [kg*m^2]

Rr = 0.697933333333333; % rotor resistance, referred to the stator side [ohm]
Rs = 1.24184;           % stator resistance [ohm]

p = 1; % pole pairs

%% Control parameters

s = tf('s');

G = K*K / (s * J * Rr);

Kp = 0.10692;
Ki = 0.19812;
Ka = 1 / Kp; % anti-windup gain
C_PI = Kp + Ki/s;
% C_PI = C_Design1

Kp_PID = 0.0924;
Ki_PID = 0.0999;
Kd_PID = 3.565e-3;
N = 7.444;
C_PID = Kp_PID + Ki_PID/s + Kd_PID*(N/(1+N/s));

omegaSlipBottomLim = -0.05 * omega_n;
omegaSlipTopLim = 0.05 * omega_n;

% Set the sampling time incase someone wants to run the Simulink model
% outside of the scripts
Ts = 1/5e3; 

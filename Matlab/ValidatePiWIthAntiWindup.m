close all; clear;

run SetupParams.m

%% Simulink Simulation
Ts = 1/5e3;
out = sim('PI_AntiWindup.slx',5);

% Extract Simulation Outputs
t = out.logsout{1}.Values.Time;
y = out.logsout{1}.Values.Data;
u = out.logsout{2}.Values.Data;
r = out.logsout{3}.Values.Data;

%% Discrete Algorithm
uDiscrete = determinePIoutputs(Kp, Ki, Ts, r-y, omegaSlipBottomLim, omegaSlipTopLim, Ka);

%% Compare Results

figure
plot(t,r,'k', t, y)
title('Plant Response with Anti-Windup')
leg = legend('Reference', 'Output');
set(leg, 'Interpreter', 'latex', 'location', 'northeast');
ylabel('Plant Output Error')
xlabel('time t')

figure
subplot(2,1,1)
plot(t,u, t,uDiscrete)
title('Control output with anti-windup. Ts = 200 us');
leg = legend('Simulink', 'Discrete');
set(leg, 'Interpreter', 'latex', 'location', 'southeast');
ylabel('Control Output u')
subplot(2,1,2)
plot(t,u-uDiscrete)
ylabel('Control Output Error')
xlabel('time t')
set(gcf,'Position',[100 100 600 600])

%% Generate test cases output
Ts = 0.1;
outTest = sim('PI_AntiWindup.slx',2);

t = outTest.logsout{1}.Values.Time;
y = outTest.logsout{1}.Values.Data;
u = outTest.logsout{2}.Values.Data;
r = outTest.logsout{3}.Values.Data;

% The errTest and uDiscreteTest arrays should be coppied and used in the C++ test cases
errTest = r-y;
uDiscreteTest = determinePIoutputs(Kp, Ki, Ts, errTest, omegaSlipBottomLim, omegaSlipTopLim, Ka);

%% Functions

function uArr = determinePIoutputs(Kp, Ki, Ts, errArr, bottomLim, topLim, Ka)
    retArr = zeros(length(errArr), 1);
    PI_WithAntiWindup(Kp, Ki, Ts, 0, bottomLim, topLim, Ka, true);
    for i = 1:length(errArr)
        retArr(i) = PI_WithAntiWindup(Kp, Ki, Ts, errArr(i), bottomLim, topLim, Ka, false);
    end
    uArr = retArr;
end

function u = PI_WithAntiWindup(Kp, Ki, Ts, err, bottomLim, topLim, Ka, resetPersistent)
    persistent windupErrCompensateVal;
    if (resetPersistent == true)
        windupErrCompensateVal = 0;
        u = 0;
        PI_Controller(Kp, Ki, Ts, err, windupErrCompensateVal, true);
    else
        u = PI_Controller(Kp, Ki, Ts, err, windupErrCompensateVal, false);
        uLim = u;
        if (u < bottomLim)
            uLim = bottomLim;
        elseif (u > topLim)
            uLim = topLim;
        end
        windupErrCompensateVal = Ka * (u - uLim);
        u = uLim;
    end
end

function u = PI_Controller(Kp, Ki, Ts, err, windupErrCompensateVal, resetPersistent)
    persistent integral;
    persistent integralInputPrev;
    if (resetPersistent == true)
        integral = 0;
        integralInputPrev = 0;
        u = 0;
    else
        proportional = Kp * err;
        % Trapezoidal
        integralInput = err - windupErrCompensateVal;
        integral = integral + Ki * (Ts/2) * (integralInput + integralInputPrev);
        integralInputPrev = integralInput;
        
        u = proportional + integral;
    end
end

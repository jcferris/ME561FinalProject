%%
% Jamie Ferris
% ME 561
% Final Project

%% Initial Setup

addpath('C:\Jamie\ZWIER\Balance Features Codes\')

% Baseline values
timeNoise = 100; %s
Kn = 462; %Nm
thetaRef = 0; %degrees
g = -9.81;

% Using values of the average adult male as outlined in [1]
mB = 76; %kg
dB = 0.87; %m
JB = 66; %kg/m^2

% Note: No passive feedback because [1] found the model to fit better
% without passive feedback

%% Part 1: Continuous, only natural feedback

% Using values of the average healthy young person as outlined in [1]

Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg

timeDelay = 0.171; %s

continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
sqrt(var(disturbance))
figure(1)
plot(time,distance*1000)
%title('Continuous Feedback, No Visual Augmentation, Young Men')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,10,txt)
%saveas(figure(1), 'ContinuousFeedback_NoVisualAugmentation_YoungMen.jpg')

% Find max velocity:
maxControlOutput = max(abs(controlOutput))

rms = RMSCal(distance, mean(distance))
maxDisp = max(abs(distance))
% Matches result from paper (within reasonable variation due to noise)


% Different time delays without reweighting
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg

timeDelay = 0.171 * 176/161; %older adult
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
sqrt(var(disturbance))
figure(1)
plot(time,distance*1000)
title('Continuous Feedback, No Visual Augmentation, Older Adult')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'ContinuousFeedback_NoVisualAugmentation_OlderAdult.jpg')

timeDelay = 0.171 * 161/102; %MS
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
sqrt(var(disturbance))
figure(1)
plot(time,distance*1000)
title('Continuous Feedback, No Visual Augmentation, Multiple Sclerosis')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,10,txt)
saveas(figure(1), 'ContinuousFeedback_NoVisualAugmentation_MS.jpg')

timeDelay = 0.171 * 47.5/44.5; %Parkinson's
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
sqrt(var(disturbance))
figure(1)
plot(time,distance*1000)
title('Continuous Feedback, No Visual Augmentation, Parkinsons Disease')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'ContinuousFeedback_NoVisualAugmentation_Parkinsons.jpg')

%% Continuous, no visual feedback, optimized

timeDelay = 0.171; % Young men
rmsMat1 = [];
ijkMat1 = [];
for i = 510:5:530
    for j = 450:5:470
        for k = 3000:500:4000
            Kp = i;
            Kd = j;
            Ki = k;
            
            continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
            time = continuousNoFbSim.yout{1}.Values.Time;
            theta = continuousNoFbSim.yout{1}.Values.Data;
            controlOutput = continuousNoFbSim.yout{2}.Values.Data;
            disturbance = continuousNoFbSim.yout{3}.Values.Data;
            distance = continuousNoFbSim.yout{4}.Values.Data;
            rmsMat1 = [rmsMat1; 1000*RMSCal(distance, mean(distance))];
            ijkMat1 = [ijkMat1; i j k];
        end
    end
end
optimal1 = ijkMat1(find(rmsMat1 == min(rmsMat1)),:)

Kp = optimal1(1);
Kd = optimal1(2);
Ki = optimal1(3);

continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
rms1 = 1000*RMSCal(distance, mean(distance))
max1 = max(abs(distance))
figure(1)
plot(time,distance*1000)

timeDelay = 0.171 * 176/161; %older adult (other paper)
rmsMat2 = [];
ijkMat2 = [];
for i = 360:5:380
    for j = 400:5:430
        for k = 2500:500:3500
            Kp = i;
            Kd = j;
            Ki = k;
            
            continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
            time = continuousNoFbSim.yout{1}.Values.Time;
            theta = continuousNoFbSim.yout{1}.Values.Data;
            controlOutput = continuousNoFbSim.yout{2}.Values.Data;
            disturbance = continuousNoFbSim.yout{3}.Values.Data;
            distance = continuousNoFbSim.yout{4}.Values.Data;
            rmsMat2 = [rmsMat2; 1000*RMSCal(distance, mean(distance))];
            ijkMat2 = [ijkMat2; i j k];
        end
    end
end
optimal2 = ijkMat2(find(rmsMat2 == min(rmsMat2)),:)

Kp = optimal2(1);
Kd = optimal2(2);
Ki = optimal2(3);

continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
rms2 = 1000*RMSCal(distance, mean(distance))
max2 = max(abs(distance))

timeDelay = 0.171 * 161/102; %MS
rmsMat3 = [];
ijkMat3 = [];
for i = 5:5:35
    for j = 265:5:295
        for k = 1000:500:2000
            Kp = i;
            Kd = j;
            Ki = k;
            
            continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
            time = continuousNoFbSim.yout{1}.Values.Time;
            theta = continuousNoFbSim.yout{1}.Values.Data;
            controlOutput = continuousNoFbSim.yout{2}.Values.Data;
            disturbance = continuousNoFbSim.yout{3}.Values.Data;
            distance = continuousNoFbSim.yout{4}.Values.Data;
            rmsMat3 = [rmsMat3; 1000*RMSCal(distance, mean(distance))];
            ijkMat3 = [ijkMat3; i j k];
        end
    end
end
optimal3 = ijkMat3(find(rmsMat3 == min(rmsMat3)),:)

Kp = optimal3(1);
Kd = optimal3(2);
Ki = optimal3(3);

continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
rms3 = 1000*RMSCal(distance, mean(distance))
max3 = max(abs(distance))

timeDelay = 0.171 * 47.5/44.5; %Parkinson's
rmsMat4 = [];
ijkMat4 = [];
for i = 400:5:410
    for j = 415:5:425
        for k = 3000:500:3500
            Kp = i;
            Kd = j;
            Ki = k;
            
            continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
            time = continuousNoFbSim.yout{1}.Values.Time;
            theta = continuousNoFbSim.yout{1}.Values.Data;
            controlOutput = continuousNoFbSim.yout{2}.Values.Data;
            disturbance = continuousNoFbSim.yout{3}.Values.Data;
            distance = continuousNoFbSim.yout{4}.Values.Data;
            rmsMat4 = [rmsMat4; 1000*RMSCal(distance, mean(distance))];
            ijkMat4 = [ijkMat4; i j k];
        end
    end
end
optimal4 = ijkMat4(find(rmsMat4 == min(rmsMat4)),:)

Kp = optimal4(1);
Kd = optimal4(2);
Ki = optimal4(3);

continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFbOptimized',800)
time = continuousNoFbSim.yout{1}.Values.Time;
theta = continuousNoFbSim.yout{1}.Values.Data;
controlOutput = continuousNoFbSim.yout{2}.Values.Data;
disturbance = continuousNoFbSim.yout{3}.Values.Data;
distance = continuousNoFbSim.yout{4}.Values.Data;
rms4 = 1000*RMSCal(distance, mean(distance))
max4 = max(abs(distance))

%% Plot continuous no feedback plots for young adult and MS

Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg

timeDelay = 0.171; % Young men
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
timeYA = continuousNoFbSim.yout{1}.Values.Time;
distanceYA = continuousNoFbSim.yout{4}.Values.Data;

timeDelay = 0.171 * 161/102; %MS
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
timeMS = continuousNoFbSim.yout{1}.Values.Time;
distanceMS = continuousNoFbSim.yout{4}.Values.Data;

Kp = optimal1(1);
Kd = optimal1(2);
Ki = optimal1(3);

timeDelay = 0.171; % Young men
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
timeYATuned = continuousNoFbSim.yout{1}.Values.Time;
distanceYATuned = continuousNoFbSim.yout{4}.Values.Data;

Kp = optimal3(1);
Kd = optimal3(2);
Ki = optimal3(3);

timeDelay = 0.171 * 161/102; %MS
continuousNoFbSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
timeMSTuned = continuousNoFbSim.yout{1}.Values.Time;
distanceMSTuned = continuousNoFbSim.yout{4}.Values.Data;

figure(1)
tiledlayout(2,2)
nexttile
plot(timeYA, distanceYA*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')
title('(A)')
nexttile
plot(timeYATuned, distanceYATuned*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')
ylim([-2,2])
title('(B)')
nexttile
plot(timeMS, distanceMS*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')
title('(C)')
nexttile
plot(timeMSTuned, distanceMSTuned*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')
ylim([-2,2])
title('(D)')

%% Part 2: Continuous, visual feedback, optimized visual fb - first sampling rate

%samplingTime = 0.01;
samplingTime = 1;
%samplingTime = 30;

timeDelay = 0.171; % Young men
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
rmsMat5 = [];
ijkMat5 = [];
for i = 100:20:300
    for j = 50:20:150
        for k = 0:1000:3000
            KpFb = i;
            KdFb = j;
            KiFb = k;
            
            continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
            time = continuousFbSim.yout{1}.Values.Time;
            theta = continuousFbSim.yout{1}.Values.Data;
            controlOutput = continuousFbSim.yout{2}.Values.Data;
            disturbance = continuousFbSim.yout{3}.Values.Data;
            distance = continuousFbSim.yout{4}.Values.Data;
            rmsMat5 = [rmsMat5; 1000*RMSCal(distance, mean(distance))];
            ijkMat5 = [ijkMat5; i j k];
        end
    end
end
optimal5 = ijkMat5(find(rmsMat5 == min(rmsMat5)),:)

KpFb = optimal5(1);
KdFb = optimal5(2);
KiFb = optimal5(3);

continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
timeYAFb = continuousFbSim.yout{1}.Values.Time;
theta = continuousFbSim.yout{1}.Values.Data;
controlOutput = continuousFbSim.yout{2}.Values.Data;
disturbance = continuousFbSim.yout{3}.Values.Data;
distance = continuousFbSim.yout{4}.Values.Data;
distanceYAFb = distance;
rms5 = 1000*RMSCal(distance, mean(distance))
max5 = 1000*max(abs(distance))
rms5b = 1000*RMSCal(distance, mean(distance))*3.76/0.242
max5b = 1000*max(abs(distance))*11.3/0.966

%%
timeDelay = 0.171 * 176/161; %older adult (other paper)
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
rmsMat6 = [];
ijkMat6 = [];
for i = 340
    for j = 600
        for k = 3000
            KpFb = i;
            KdFb = j;
            KiFb = k;
            
            continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
            time = continuousFbSim.yout{1}.Values.Time;
            theta = continuousFbSim.yout{1}.Values.Data;
            controlOutput = continuousFbSim.yout{2}.Values.Data;
            disturbance = continuousFbSim.yout{3}.Values.Data;
            distance = continuousFbSim.yout{4}.Values.Data;
            rmsMat6 = [rmsMat6; 1000*RMSCal(distance, mean(distance))];
            ijkMat6 = [ijkMat6; i j k];
        end
    end
end
optimal6 = ijkMat6(find(rmsMat6 == min(rmsMat6)),:)

KpFb = optimal6(1);
KdFb = optimal6(2);
KiFb = optimal6(3);

continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
timeOAFB = continuousFbSim.yout{1}.Values.Time;
theta = continuousFbSim.yout{1}.Values.Data;
controlOutput = continuousFbSim.yout{2}.Values.Data;
disturbance = continuousFbSim.yout{3}.Values.Data;
distance = continuousFbSim.yout{4}.Values.Data;
distanceOAFb = distance;
rms6 = 1000*RMSCal(distance, mean(distance))
max6 = 1000*max(abs(distance))
rms6b = 1000*RMSCal(distance, mean(distance))*3.76/0.242
max6b = 1000*max(abs(distance))*11.3/0.966
%%
timeDelay = 0.171 * 161/102; %MS
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
rmsMat7 = [];
ijkMat7 = [];
for i = 5:5:15
    for j = 1085:5:1095
        for k = 0000:1000:2000
            KpFb = i;
            KdFb = j;
            KiFb = k;
            
            continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
            time = continuousFbSim.yout{1}.Values.Time;
            theta = continuousFbSim.yout{1}.Values.Data;
            controlOutput = continuousFbSim.yout{2}.Values.Data;
            disturbance = continuousFbSim.yout{3}.Values.Data;
            distance = continuousFbSim.yout{4}.Values.Data;
            rmsMat7 = [rmsMat7; 1000*RMSCal(distance, mean(distance))];
            ijkMat7 = [ijkMat7; i j k];
        end
    end
end
optimal7 = ijkMat7(find(rmsMat7 == min(rmsMat7)),:)

KpFb = optimal7(1);
KdFb = optimal7(2);
KiFb = optimal7(3);

continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
timeMSFB = continuousFbSim.yout{1}.Values.Time;
theta = continuousFbSim.yout{1}.Values.Data;
controlOutput = continuousFbSim.yout{2}.Values.Data;
disturbance = continuousFbSim.yout{3}.Values.Data;
distance = continuousFbSim.yout{4}.Values.Data;
distanceMSFb = distance;
rms7 = 1000*RMSCal(distance, mean(distance))
max7 = 1000*max(abs(distance))
rms7b = 1000*RMSCal(distance, mean(distance))*3.76/0.242
max7b = 1000*max(abs(distance))*11.3/0.966
%%
timeDelay = 0.171 * 47.5/44.5; %Parkinson's
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
rmsMat8 = [];
ijkMat8 = [];
for i = 140:5:160
    for j = 35:5:45
        for k = 0000:1000:1000
            KpFb = i;
            KdFb = j;
            KiFb = k;
            
            continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
            time = continuousFbSim.yout{1}.Values.Time;
            theta = continuousFbSim.yout{1}.Values.Data;
            controlOutput = continuousFbSim.yout{2}.Values.Data;
            disturbance = continuousFbSim.yout{3}.Values.Data;
            distance = continuousFbSim.yout{4}.Values.Data;
            rmsMat8 = [rmsMat8; 1000*RMSCal(distance, mean(distance))];
            ijkMat8 = [ijkMat8; i j k];
        end
    end
end
optimal8 = ijkMat8(find(rmsMat8 == min(rmsMat8)),:)

KpFbFb = optimal8(1);
KdFb = optimal8(2);
KiFbFb = optimal8(3);

continuousFbSim = sim('ME561FinalProjectSimulink_ContinuousFbOptimized',800)
timePFB = continuousFbSim.yout{1}.Values.Time;
theta = continuousFbSim.yout{1}.Values.Data;
controlOutput = continuousFbSim.yout{2}.Values.Data;
disturbance = continuousFbSim.yout{3}.Values.Data;
distance = continuousFbSim.yout{4}.Values.Data;
distancePFb = distance;
rms8 = 1000*RMSCal(distance, mean(distance))
max8 = 1000*max(abs(distance))
rms8b = 1000*RMSCal(distance, mean(distance))*3.76/0.242
max8b = 1000*max(abs(distance))*11.3/0.966


%% Part 3: Discrete, no visual feedback

timeDelay = 0.171; %s
humanSamplingTime = 0.01;
feedbackSamplingTime = 0.01;

Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
K = Kp;
Ti = K/Ki;
Td = Kd/K;
T = humanSamplingTime;
KpDiscrete = K - K*T/(2*Ti); %Nm/deg
KiDiscrete = K*T/Ti; %Nms/deg
KdDiscrete = K*Td/T; %Nm/sdeg

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
%title('Discrete Feedback, No Visual Augmentation, Young Men')
xlabel('Time (s)')
ylabel('Displacement (mm)')
%txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
%text(500,1,txt)
%saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_YoungMen.jpg')

%Try with manual delay
discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized_Delay',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(2)
plot(time,distance*1000)
RMSCal(distance, mean(distance))
% The difference is there, but very small. There is no difference if the
% delay is 0.18 s (an increment of the sampling time)

%% For other subject populations

timeDelay = 0.171 * 176/161; %older adult (other paper)
discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')

timeDelay = 0.171 * 161/102; %MS
discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')

timeDelay = 0.171 * 47.5/44.5; %Parkinson's
discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
xlabel('Time (s)')
ylabel('Displacement (mm)')


%% Discrete, no visual feedback, optimized

timeDelay = 0.171; %s
humanSamplingTime = 0.01;
feedbackSamplingTime = 0.01;

KpDiscrete = 0;
KdDiscrete = 5000;
KiDiscrete = 2.4;

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
title('Discrete Feedback, No Visual Augmentation, Young Men, Tuned')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_YoungMen_Tuned.jpg')

%% For other subject populations

timeDelay = 0.171; % Young men
rmsMat1 = [];
ijkMat1 = [];
for i = 0
    for j = 9000
        for k = 3.5:0.1:5
            KpDiscrete = i;
            KdDiscrete = j;
            KiDiscrete = k;
            
            discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
            time = discreteNoFbSim.yout{1}.Values.Time;
            theta = discreteNoFbSim.yout{1}.Values.Data;
            controlOutput = discreteNoFbSim.yout{2}.Values.Data;
            disturbance = discreteNoFbSim.yout{3}.Values.Data;
            distance = discreteNoFbSim.yout{4}.Values.Data;
            rmsMat1 = [rmsMat1; 1000*RMSCal(distance, mean(distance))];
            ijkMat1 = [ijkMat1; i j k];
        end
    end
end
optimal1 = ijkMat1(find(rmsMat1 == min(rmsMat1)),:)

KpDiscrete = optimal1(1);
KdDiscrete = optimal1(2);
KiDiscrete = optimal1(3);

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
rms1 = 1000*RMSCal(distance, mean(distance))
max1 = max(abs(distance))

timeDelay = 0.171 * 176/161; %older adult (other paper)
rmsMat2 = [];
ijkMat2 = [];
for i = 0
    for j = 9000
        for k = 3.5:0.1:5
            KpDiscrete = i;
            KdDiscrete = j;
            KiDiscrete = k;
            
            discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
            time = discreteNoFbSim.yout{1}.Values.Time;
            theta = discreteNoFbSim.yout{1}.Values.Data;
            controlOutput = discreteNoFbSim.yout{2}.Values.Data;
            disturbance = discreteNoFbSim.yout{3}.Values.Data;
            distance = discreteNoFbSim.yout{4}.Values.Data;
            rmsMat2 = [rmsMat2; 1000*RMSCal(distance, mean(distance))];
            ijkMat2 = [ijkMat2; i j k];
        end
    end
end
optimal2 = ijkMat2(find(rmsMat2 == min(rmsMat2)),:)

KpDiscrete = optimal2(1);
KdDiscrete = optimal2(2);
KiDiscrete = optimal2(3);

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
rms2 = 1000*RMSCal(distance, mean(distance))
max2 = max(abs(distance))

timeDelay = 0.171 * 161/102; %MS
rmsMat3 = [];
ijkMat3 = [];
for i = 0
    for j = 9000
        for k = 3.5:0.1:5
            KpDiscrete = i;
            KdDiscrete = j;
            KiDiscrete = k;
            
            discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
            time = discreteNoFbSim.yout{1}.Values.Time;
            theta = discreteNoFbSim.yout{1}.Values.Data;
            controlOutput = discreteNoFbSim.yout{2}.Values.Data;
            disturbance = discreteNoFbSim.yout{3}.Values.Data;
            distance = discreteNoFbSim.yout{4}.Values.Data;
            rmsMat3 = [rmsMat3; 1000*RMSCal(distance, mean(distance))];
            ijkMat3 = [ijkMat3; i j k];
        end
    end
end
optimal3 = ijkMat3(find(rmsMat3 == min(rmsMat3)),:)

KpDiscrete = optimal3(1);
KdDiscrete = optimal3(2);
KiDiscrete = optimal3(3);

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
rms3 = 1000*RMSCal(distance, mean(distance))
max3 = max(abs(distance))

timeDelay = 0.171 * 47.5/44.5; %Parkinson's
rmsMat4 = [];
ijkMat4 = [];
for i = 0
    for j = 9000
        for k = 3.5:0.1:5
            KpDiscrete = i;
            KdDiscrete = j;
            KiDiscrete = k;
            
            discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
            time = discreteNoFbSim.yout{1}.Values.Time;
            theta = discreteNoFbSim.yout{1}.Values.Data;
            controlOutput = discreteNoFbSim.yout{2}.Values.Data;
            disturbance = discreteNoFbSim.yout{3}.Values.Data;
            distance = discreteNoFbSim.yout{4}.Values.Data;
            rmsMat4 = [rmsMat4; 1000*RMSCal(distance, mean(distance))];
            ijkMat4 = [ijkMat4; i j k];
        end
    end
end
optimal4 = ijkMat4(find(rmsMat4 == min(rmsMat4)),:)

KpDiscrete = optimal4(1);
KdDiscrete = optimal4(2);
KiDiscrete = optimal4(3);

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
rms4 = 1000*RMSCal(distance, mean(distance))
max4 = max(abs(distance))

%% Part 4: Discrete, visual feedback, half tuned using tuning from continuous case

timeDelay = 0.171; %s
humanSamplingTime = 0.01;
feedbackSamplingTime = 0.01;

Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
K = Kp;
Ti = K/Ki;
Td = Kd/K;
T = humanSamplingTime;
KpDiscrete = K - K*T/(2*Ti); %Nm/deg
KiDiscrete = K*T/Ti; %Nms/deg
KdDiscrete = K*Td/T; %Nm/sdeg

% Young Men

KpFb = 0.1; %Nm/deg
KdFb = 1000; %Nms/deg
KiFb = 1000; %Nm/sdeg
KFb = KpFb;
TiFb = KFb/KiFb;
TdFb = KdFb/KFb;
TFb = humanSamplingTime;
KpFbDiscrete = KFb - KFb*TFb/(2*TiFb); %Nm/deg
KiFbDiscrete = KFb*TFb/TiFb; %Nms/deg
KdFbDiscrete = KFb*TdFb/TFb; %Nm/sdeg

timeDelay = 0.171;
discreteFbSim = sim('ME561FinalProjectSimulink_DiscreteFbOptimized',800)
time = discreteFbSim.yout{1}.Values.Time;
theta = discreteFbSim.yout{1}.Values.Data;
controlOutput = discreteFbSim.yout{2}.Values.Data;
disturbance = discreteFbSim.yout{3}.Values.Data;
distance = discreteFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
title('Discrete Feedback, Visual Augmentation, Young Men, Half Tuned')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_YoungMen_HalfTuned.jpg')

% Older adult

KpFb = 0.1; %Nm/deg
KdFb = 9000; %Nms/deg
KiFb = 4350; %Nm/sdeg
KFb = KpFb;
TiFb = KFb/KiFb;
TdFb = KdFb/KFb;
TFb = humanSamplingTime;
KpFbDiscrete = KFb - KFb*TFb/(2*TiFb); %Nm/deg
KiFbDiscrete = KFb*TFb/TiFb; %Nms/deg
KdFbDiscrete = KFb*TdFb/TFb; %Nm/sdeg

timeDelay = 0.171 * 176/161; %older adult (other paper)
discreteFbSim = sim('ME561FinalProjectSimulink_DiscreteFbOptimized',800)
time = discreteFbSim.yout{1}.Values.Time;
theta = discreteFbSim.yout{1}.Values.Data;
controlOutput = discreteFbSim.yout{2}.Values.Data;
disturbance = discreteFbSim.yout{3}.Values.Data;
distance = discreteFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
title('Discrete Feedback, Visual Augmentation, Older Adult, Half Tuned')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_OlderAdult_HalfTuned.jpg')

% MS

KpFb = -35; %Nm/deg
KdFb = 8000; %Nms/deg
KiFb = 1340; %Nm/sdeg
KFb = KpFb;
TiFb = KFb/KiFb;
TdFb = KdFb/KFb;
TFb = humanSamplingTime;
KpFbDiscrete = KFb - KFb*TFb/(2*TiFb); %Nm/deg
KiFbDiscrete = KFb*TFb/TiFb; %Nms/deg
KdFbDiscrete = KFb*TdFb/TFb; %Nm/sdeg

timeDelay = 0.171 * 161/102; %MS
discreteFbSim = sim('ME561FinalProjectSimulink_DiscreteFbOptimized',800)
time = discreteFbSim.yout{1}.Values.Time;
theta = discreteFbSim.yout{1}.Values.Data;
controlOutput = discreteFbSim.yout{2}.Values.Data;
disturbance = discreteFbSim.yout{3}.Values.Data;
distance = discreteFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
title('Discrete Feedback, Visual Augmentation, Multiple Sclerosis, Half Tuned')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_MultipleSclerosis_HalfTuned.jpg')

% Parkinson's Disease

KpFb = 0.1; %Nm/deg
KdFb = 8000; %Nms/deg
KiFb = 1000; %Nm/sdeg
KFb = KpFb;
TiFb = KFb/KiFb;
TdFb = KdFb/KFb;
TFb = humanSamplingTime;
KpFbDiscrete = KFb - KFb*TFb/(2*TiFb); %Nm/deg
KiFbDiscrete = KFb*TFb/TiFb; %Nms/deg
KdFbDiscrete = KFb*TdFb/TFb; %Nm/sdeg

timeDelay = 0.171 * 47.5/44.5; %Parkinson's
discreteFbSim = sim('ME561FinalProjectSimulink_DiscreteFbOptimized',800)
time = discreteFbSim.yout{1}.Values.Time;
theta = discreteFbSim.yout{1}.Values.Data;
controlOutput = discreteFbSim.yout{2}.Values.Data;
disturbance = discreteFbSim.yout{3}.Values.Data;
distance = discreteFbSim.yout{4}.Values.Data;
figure(1)
plot(time,distance*1000)
title('Discrete Feedback, Visual Augmentation, Parkinsons, Half Tuned')
xlabel('Time (s)')
ylabel('Displacement (mm)')
txt = {'RMS Displacement:', strcat(string(1000*RMSCal(distance, mean(distance))), ' mm')}; 
text(500,1,txt)
saveas(figure(1), 'DiscreteFeedback_NoVisualAugmentation_Parkinsons_HalfTuned.jpg')


%% Part 5: Analyzing stability in fully discrete system

% Checking assumptions

%% 1. Manual delay using z^-N
discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized_Delay',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(2)
plot(time,distance*1000)
RMSCal(distance, mean(distance))
% The difference is there, but very small. There is no difference if the
% delay is 0.18 s (an increment of the sampling time)

%% 2. Model noise as a collection of sines and cosines

% Find noise function after filter

noiseSim = sim('ME561FinalProjectSimulink_ContinuousNoFb',800)
time = noiseSim.yout{1}.Values.Time;
noise = noiseSim.yout{3}.Values.Data;
Y = fft(noise);
L = length(noise);
Fs = 1/0.01;
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
figure(1)
plot(time, noise)
figure(2)
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
axis([0 0.2 0 3])
var(Y)

A = 1.03;
t = time;
approxNoise = A*3*sin(2*pi*0.001*t) + A*2*sin(2*pi*0.006*t) + A*1*sin(2*pi*0.01*t) + A*1*sin(2*pi*0.02*t) + A*0.75*sin(2*pi*0.04*t) + A*0.6*sin(2*pi*0.1*t) + A*0.5*sin(2*pi*0.4*t) + A*0.5*sin(2*pi*0.5*t);
Y = fft(approxNoise);
L = length(noise);
Fs = 1/0.01;
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
figure(3)
plot(time, approxNoise)
figure(4)
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
axis([0 0.2 0 3])
var(Y)

% Compare output with sine approximated noise, standard pid, no feedback

timeDelay = 0.171; %s
humanSamplingTime = 0.01;
feedbackSamplingTime = 0.01;

Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
K = Kp;
Ti = K/Ki;
Td = Kd/K;
T = humanSamplingTime;
KpDiscrete = K - K*T/(2*Ti); %Nm/deg
KiDiscrete = K*T/Ti; %Nms/deg
KdDiscrete = K*Td/T; %Nm/sdeg

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(5)
plot(time,distance*1000)
RMSCal(distance, mean(distance))

discreteNoFbSim = sim('ME561FinalProjectSimulink_DiscreteNoFbOptimized_Simple',800)
time = discreteNoFbSim.yout{1}.Values.Time;
theta = discreteNoFbSim.yout{1}.Values.Data;
controlOutput = discreteNoFbSim.yout{2}.Values.Data;
disturbance = discreteNoFbSim.yout{3}.Values.Data;
distance = discreteNoFbSim.yout{4}.Values.Data;
figure(6)
plot(time,distance*1000)
RMSCal(distance, mean(distance))

% Approximate match, so use approxNoise function as input

%% Calculate stability for continuous, no fb, not tuned

% For young men, not tuned
% Note: because delay is not included, counts for all subject populations
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg

sys = tf([Kd Kp Ki],[JB 0 -mB*g*dB 0], humanSamplingTime)
figure(2)
rlocus(sys)
title('Root Locus')

% Find result of gain 1
[complexVec,gainsVec] = rlocus(sys, [1]);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
max(magnitude)
% Unstable
% Verified using Simulink model

% Find 
[complexVec,gainsVec] = rlocus(sys);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
maxMag = max(magnitude);
find(maxMag<1)
% Never stable

%% Calculate stability for continuous, no fb, tuned

% For young men, tuned
% Note: because delay is not included, counts for all subject populations
Kp = 530; %Nm/deg
Kd = 455; %Nms/deg
Ki = 3500; %Nm/sdeg

sys = tf([Kd Kp Ki],[JB 0 -mB*g*dB 0], humanSamplingTime)
figure(2)
rlocus(sys)
title('(A)')

% Find result of gain 1
[complexVec,gainsVec] = rlocus(sys, [1]);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
max(magnitude)
% Unstable
% Verified using Simulink model

% Find 
[complexVec,gainsVec] = rlocus(sys);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
maxMag = max(magnitude);
find(maxMag<1)
% Never stable

%% Calculate stability for discrete, no fb, not tuned

% For young men, not tuned
Kp = 16.7; %Nm/deg
Kd = 4.83; %Nms/deg
Ki = 0.60; %Nm/sdeg
K = Kp;
Ti = K/Ki;
Td = Kd/K;
T = humanSamplingTime;
KpDiscrete = K - K*T/(2*Ti); %Nm/deg
KiDiscrete = K*T/Ti; %Nms/deg
KdDiscrete = K*Td/T; %Nm/sdeg

sys = tf([-Kp-Ki-Kd Kp+2*Kd -Kd],[-1/JB 3/JB -3/JB 1/JB 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -mB*g*dB 0], humanSamplingTime)
figure(2)
rlocus(sys)
title('(A)')

% Find result of gain 1
[complexVec,gainsVec] = rlocus(sys, [1]);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
max(magnitude)
% Unstable
% Verified using Simulink model

% Find 
[complexVec,gainsVec] = rlocus(sys);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
maxMag = max(magnitude);
find(maxMag<1)
% Never stable

%%
% For young men, tuned

Kp = 450; %Nm/deg
Kd = 195; %Nms/deg
Ki = 3000; %Nm/sdeg
K = Kp;
Ti = K/Ki;
Td = Kd/K;
T = humanSamplingTime;
KpDiscrete = K - K*T/(2*Ti); %Nm/deg
KiDiscrete = K*T/Ti; %Nms/deg
KdDiscrete = K*Td/T; %Nm/sdeg

sys = tf([-Kp-Ki-Kd Kp+2*Kd -Kd],[-1/JB 3/JB -3/JB 1/JB 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -mB*g*dB 0], humanSamplingTime)
figure(3)
rlocus(sys)
title('(B)')

% Find result of gain 1
[complexVec,gainsVec] = rlocus(sys, [1]);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
max(magnitude)
% Unstable

% Find 
[complexVec,gainsVec] = rlocus(sys);
realPart = real(complexVec);
imagPart = imag(complexVec);
magnitude = (realPart.^2 + imagPart.^2).^0.5;
maxMag = max(magnitude);
find(maxMag<1)
% Never stable

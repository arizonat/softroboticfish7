%% Create signal
ball_f = 0.1;
swim_f = 1.25;
sample_f = 24;

ball_T = 1/ball_f;
swim_T = 1/swim_f;
sample_T = 1/sample_f;

tf = 30; % seconds

ts = 0:sample_T:tf;
y_swim = 0.2*sin(2*pi*swim_f*ts + 2*pi*swim_f*2.5);
y_ball = sin(2*pi*ball_f*ts + 2*pi*ball_f*2.5);
y = y_swim + y_ball; % y is the observed signal

%% Attempt to estimate signal
cutoff_f = 1.25;%ball_f*11;%ball_f*5; %swim_f*0.5;

% Setup simple IIR filter
y_diir = y(1); %exponential smoothing or infinite impulse response filter
RC = 1/(2*pi*(cutoff_f));
alpha = sample_T/(RC+sample_T);

% Setup simple FIR filter
y_fir = y(1); %moving windowed average, evenly weighted, or finite impulse response
%win = floor(swim_T/sample_T);
%win = floor((1/cutoff_f)/(sample_T*2));
win = 20;

% Setup simple butterworth filter (used by the PID ROS node on the D-term?)
order = 2;
[bb, ba] = butter(order, cutoff_f/(sample_f/2));
[y_b, z] = filter(bb, ba, y(1), zeros(1,order));

% Setup Braden filter
[y_br, m, y_min, y_max] = braden_filter(y(1), 0, 0, y(1), y(1));

% Setup sampled Braden filter
sampler_f = swim_f*2;
sampler_T = 1/sampler_f;
t0 = ts(1);
x0 = y(1);
x1 = y(1);
y_sbr = x0;
yt_sbr = x0;

for t=2:size(ts,2)
    dyt = y(t) - y(t-1);
    
    % Update simple IIR filter
    yt = alpha * y(t) + (1-alpha) * y_diir(t-1);
    y_diir = [y_diir yt];
    
    % Update simple FIR filter
    yt_fir = sum(y(max(t-win,1):t))/min(win,t);
    y_fir = [y_fir yt_fir];
    
    % Update butterworth filter
    [yt_b, z] = filter(bb, ba, y(t), z);
    y_b = [y_b yt_b];
    
    % Update Braden filter
    m_new = y(t)-y(t-1);
    [yt_br, m, y_min, y_max] = braden_filter(y(t), m_new, m, y_min, y_max);
    y_br = [y_br yt_br];
    
    % Update Sampled Braden filter
    if ts(t)-t0 > sampler_T
        xf = y(t);
        yt_sbr = mean([xf x0]);
        t0 = ts(t);
        x0 = xf;
    end
    y_sbr = [y_sbr yt_sbr];
end

disp(["Cutoff frequency: " num2str(cutoff_f)])
disp("L1 errors:")
disp(["No filter: " num2str(sum(abs(y_ball-y)))])
disp(["Exponential smoothing: " num2str(sum(abs(y_ball-y_diir)))])
disp(["Windowed average smoothing: " num2str(sum(abs(y_ball-y_fir)))])
disp(["Butterworth filter: " num2str(sum(abs(y_ball-y_b)))])
disp(["Braden filter: " num2str(sum(abs(y_ball-y_br)))])
disp(["Fixed rate Braden filter: " num2str(sum(abs(y_ball-y_sbr)))])

%% Plot
close all;
figure;
hold on;
plot(ts,y_ball);
plot(ts,y);
plot(ts,y_diir);
plot(ts,y_fir);
plot(ts,y_b);
plot(ts,y_br);
plot(ts,y_sbr);
%legend("target","signal","fir","braden","sbr")
legend("target","signal","iir","fir","butter","braden","sbr")

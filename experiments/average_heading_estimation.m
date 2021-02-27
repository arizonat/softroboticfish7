%% Create signal
ball_f = 0.1;
swim_f = 1.25;
sample_f = 24;

ball_T = 1/ball_f;
swim_T = 1/swim_f;
sample_T = 1/sample_f;

tf = 30; % seconds

x = 1:sample_T:tf;
y_swim = 0.2*sin(2*pi*swim_f*x);
y_ball = sin(2*pi*ball_f*x+10);
y = y_swim + y_ball; % y is the observed signal

%% Attempt to estimate signal
y_diir = y(1); %exponential smoothing or infinite impulse response filter
RC = 1/(2*pi*(swim_f*0.2));
alpha = sample_T/(RC+sample_T);

y_fir = y(1); %moving windowed average, evenly weighted, or finite impulse response
win = floor(swim_T/sample_T);

for t=2:size(x,2)
    dyt = y(t) - y(t-1);
    
    % Update simple IIR filter
    yt = alpha * y(t) + (1-alpha) * h(t-1);
    y_diir = [y_diir yt];
    
    % Update simple FIR filter
    yt_fir = sum(y(max(t-win,1):t))/min(win,t);
    y_fir = [y_fir yt_fir];
    
end


%% Plot
close all;
figure;
hold on;
%plot(x,y_swim);
plot(x,y_ball);
plot(x,y);
plot(x,y_diir);
plot(x,y_fir);
legend("target","signal","iir","fir")
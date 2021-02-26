ball_f = 0.1;
swim_f = 1.25;
sample_f = 24;

swim_T = 1/swim_f;
sample_T = 1/sample_f;

tf = 30; % seconds

x = 1:sample_T:tf;
y_swim = 0.2*sin(2*pi*swim_f*x);
y_ball = sin(2*pi*ball_f*x+10);
y = y_swim + y_ball;

h = y(1);
RC = 1/(2*pi*ball_f);
alpha = sample_T/(RC+sample_T);
for t=2:size(x,2)
    yt = alpha * y(t) + (1-alpha) * h(t-1);
    h = [h yt];
end

close all;
figure;
hold on;
%plot(x,y_swim);
plot(x,y_ball);
plot(x,y);
plot(x,h);
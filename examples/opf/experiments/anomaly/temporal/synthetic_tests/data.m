%% generate synthetic datasets
dpi = pi/10;
range = [0:dpi:10000]'; % from 0 to 10.000 with step PI/10

%% sin(x)
sine = sin(range); % sin(x)
sine2x = sin(2*range); % sin(2x)
sine05x = sin(0.5*range); % sin(x/2)

%% white noise
rnd = 0 + 1.*randn(size(range,1),1); % white noise with mean=0, std=1 

%% export to CSV
% needs to round to 4 decimal places
sine = round(sine*10000)/10000;
sine2x = round(sine2x*10000)/10000;
sine05x = round(sine05x*10000)/10000;
rnd = round(rnd*10000)/10000;

% write
csvwrite('./datasets/sine.csv', sine)
csvwrite('./datasets/sine2x.csv', sine2x)
csvwrite('./datasets/sine05x.csv', sine05x)
csvwrite('./datasets/rnd.csv', rnd)

%% load and plot results (from format_output.sh)
SKIP=4; % these lines contain comments (string)
res = csvread('./results/sine_10000.csv', SKIP)
TOTAL = size(res, 1);
figure()
plot(res)
xlabel('step')
ylabel('Anomaly score')
legend(['Anomaly likelihood'])
title('Anomaly on sine wave')


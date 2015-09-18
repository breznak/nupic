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
aPure = csvread('./results/sine_10000_anomaly_pure.csv', SKIP);
aLike = csvread('./results/sine_likelihood_10000.csv', SKIP);
aWeighted = csvread('./results/sine_10000_anomaly_weighted.csv', SKIP);
tmp = csvread('./results/sine_10000_pred.csv', SKIP); 
actual = tmp(:,1); % actual value, raw
pred = tmp(:,2); % predicted

TOTAL = size(aPure, 1);
figure()
plot(aPure)
hold all
plot(aLike)
plot(aWeighted)
%plot(actual)
%plot(pred)
xlabel('step')
ylabel('Anomaly score')
legend('Anomaly pure', 'Anomaly likelihood','Anomaly weighted','actual','predicted')
title('Anomaly on sine wave')



% random noise 
SKIP=4; % these lines contain comments (string)
aPure = csvread('./results/rand_10000_anomaly_pure.csv', SKIP);
aLike = csvread('./results/rand_10000_anomaly_likelihood.csv', SKIP);
aWeighted = csvread('./results/rand_10000_anomaly_weighted.csv', SKIP);
tmp = csvread('./results/rand_10000_pred.csv', SKIP); 
actual = tmp(:,1); % actual value, raw
pred = tmp(:,2); % predicted

TOTAL = size(aPure, 1);
figure()
plot(aPure)
hold all
plot(aLike)
plot(aWeighted)
%plot(actual)
%plot(pred)
xlabel('step')
ylabel('Anomaly score')
legend('Anomaly pure', 'Anomaly likelihood','Anomaly weighted','actual','predicted')
title('Anomaly on Random noise')

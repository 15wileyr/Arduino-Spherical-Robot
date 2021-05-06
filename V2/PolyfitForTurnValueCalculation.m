format long
clc
clear all
close all
x=[30000,24000,17000,10000,7000]  % Stepper pulse lengths in microseconds
y=[19000,11000,5000,2000,1000]  % Desired output turning value stepper pulse 


Coeffs=polyfit(x,y,2)

xpoly=linspace(7000,30000);
ypoly=Coeffs(1)*xpoly.^2+Coeffs(2)*xpoly+Coeffs(3);

figure(1)
plot(xpoly,ypoly)
hold on
scatter(x,y,'x')
title('Using Regression to get turning function coefficients')
xlabel('Throttle Time [micro seconds] input')
ylabel('Turn Time [micro seconds] output')
legend('Polyfit Function','Points Used to Fit')

clc;
clear;
load leleccum;
x=leleccum;
N=length(x);
y=angle(fft(x));
fs=100;
f= (1:N)*fs/N;
plot(f,y);
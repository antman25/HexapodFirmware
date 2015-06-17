function [ pwm ] = AngleToPWM( angle )
%ANGLETOPWM Summary of this function goes here
%   Detailed explanation goes here
pwm = ((angle + 90) * 10) +600;

end


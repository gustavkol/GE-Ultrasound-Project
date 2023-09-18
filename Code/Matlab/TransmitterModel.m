% Static Input parameters %
noElements  = 64;
p           = 250*10^-6;    % Pitch length(distance between transducers)
t_s         = 40*10^-9;     % Sample period
f_s         = 1/t_s;        % Sample frequency
v           = 1540;         % Speed of sound


% Pre-calculating constants for each transducer, to reduce latency in hardware 
n = -noElements/2+1:1:noElements/2;
A = (n*p/v).^2;
B = 1/(v^2);
C = (2*n*p)/(v^2);

% Variable input values
R_0 = 32*p; angle = 120*pi/180;

% Calculating squared delay for all elements %
delay_squared = -noElements/2+1:1:noElements/2;
delay = -noElements/2+1:1:noElements/2;
delta_delay = -noElements/2+1:1:noElements/2;

for i = 1:length(n)
    delay_squared(i) = A(i) + B*R_0^2 - C(i)*R_0*cos(angle);
    delay(i) = sqrt(abs(delay_squared(i)));
end

for i = 1:length(n)
    delta_delay(i) = delay(i)-delay(32);
    k(i) = n(i) * p;
end

%polarscatter(0,k,".",'black'); hold on;
polarscatter(angle, R_0,'filled','blue'); hold on;
polarplot([angle,angle], [0,R_0],'red'); hold on;

for i = 1:length(n)
    [theta,rho] = cart2pol([k(i),k(i)],[0,1000*delta_delay(i)]);
    if delta_delay(i) >= 0
        polarplot(theta,rho,'green');
    else
        polarplot(theta,rho,'red');
    end
end
hold off;

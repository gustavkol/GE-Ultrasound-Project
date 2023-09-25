% TODO: Implement CORDIC to find cos(angle) and use in iterative model
% Find square root implementation

% Static Input parameters %
noElements  = 64;
p           = 250 * 10^-6;          % Pitch length
t_s         = 40*10^-9;             % Sample period
f_s         = 1/t_s;                % Sample frequency
f_clk       = 100 * 10^6;           % Clock frequency
v           = 1540;                 % Speed of sound
n           = -31:1:32;             % Element indexes
x           = n*p;                  % x-pos of each element
delta_length= v/f_clk;              % Increment length for scanline
num_points  = 2^10;                  % Number of points on scanline
scan_length = num_points*v/f_clk;   % Length of scanline

% Variable input values
R_0 =50*p; angle = 120*pi/180;

% Calculating reference delays
a = R_0*sin(angle);
b = x - R_0*cos(angle);
R_n = sqrt(a.^2+b.^2);
delay_reference = (f_clk*R_n/v);

% Calculating squared delay for all elements %
delay_sq = noElements/2:1:noElements/2;
delta_delay = noElements/2:1:noElements/2;
delay = noElements/2:1:noElements/2;
error_delay = noElements/2:1:noElements/2;

% Pre-calculating constants for each scanline, to reduce latency in hardware
A_0 = (f_clk*p/v)^2; 
C_0 = (f_clk/v)^2 * 2 * p * R_0 * cos(angle);

% Setting values for reference element in origo
n0_index = 32;
delay_sq(n0_index) = (f_clk/v)^2*R_0^2;
delay(n0_index) = sqrt(delay_sq(n0_index));

% Iteratively calculating delay for all elements for first scan point
for i = 1:32
    cur_index = n0_index + i;
    delay_sq(cur_index) = delay_sq(cur_index-1) + A_0*(2*i+1) - C_0;
    if delay_sq(cur_index) < 0
        delay(cur_index) = -sqrt(delay_sq(cur_index));
    else
        delay(cur_index) = sqrt(delay_sq(cur_index));
    end
end
for i = 1:31
    cur_index = n0_index - i;
    delay_sq(cur_index) = delay_sq(cur_index+1) + A_0*(2*i+1) + C_0;
    if delay_sq(cur_index) < 0
        delay(cur_index) = -sqrt(delay_sq(cur_index));
    else
        delay(cur_index) = sqrt(delay_sq(cur_index));
    end
end

% Calculating difference between in delay between origo and other elements
for i = 1:length(n)
    delta_delay(i) = delay(i)-delay(n0_index);
end

% Calculating error from correct value
for i = 1:length(n)
    error_delay(i) = delay(i)-delay_reference(i);
end

% Plotting scanpoint
figure(1);
polarscatter(angle, R_0,'filled','blue'); hold on;
polarplot([angle,angle], [0,R_0],'red'); hold on;
title("Delay for each element to reference scanpoint");

% Plotting illustration of delay for each element at first point of
% scanline, error plotted in black
delay_gain = R_0*0.005;
for i = 1:length(n)-1
    [theta,rho] = cart2pol([x(i),x(i)],[0,delay_gain*delta_delay(i)]); 
    if delta_delay(i) >= 0
        polarplot(theta,rho,'green');
        [theta1,rho1] = cart2pol([x(i),x(i)],[0,delay_gain*error_delay(i)]);
    else
        polarplot(theta,rho,'red');
        [theta1,rho1] = cart2pol([x(i),x(i)],[0,-delay_gain*error_delay(i)]);
    end
    polarplot(theta1,rho1,'black');
end

% Calculating delay in next point on scanline
B_n = 2 * R_0 * (f_clk/v) - 2 * n * p * (f_clk/v) * cos(angle);
scanline_delays_sq = zeros(num_points, length(delay_sq));
scanline_delays = zeros(num_points, length(delay_sq));
scanline_reference_delays = zeros(num_points);

scanline_delays_sq(1,:) = delay_sq;
scanline_delays(1,:) = sqrt(delay_sq);
scanline_reference_delays(1) = (f_clk*R_0/v);
for k = 2:num_points
    scanline_delays_sq(k,:) = scanline_delays_sq(k-1,:) + 2*k + 1 + B_n;
    scanline_delays(k,:) = sqrt(scanline_delays_sq(k,:));
    scanline_reference_delays(k) = (f_clk*(R_0+k*delta_length)/v);
end
figure(2);
x_ax = R_0:delta_length:R_0+delta_length*(num_points-1);
plot(x_ax,scanline_delays(:,n0_index,:)); hold on
plot(x_ax,scanline_reference_delays); hold on
legend("Iterative", "Theoretical");
title("Delay for element n=0 as focal points moves through scanline");
hold off;

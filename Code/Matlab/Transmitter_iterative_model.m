% TODO: Find and implement a way to do square root in hardware

% Static memory parameters %
noElements  = 64;                   % Number of transducer elements
p           = 250 * 10^-6;          % Pitch length
t_s         = 40*10^-9;             % Sample period
f_s         = 1/t_s;                % Sample frequency
f_clk       = 100 * 10^6;           % Clock frequency
v           = 1540;                 % Speed of sound
n           = -31:1:32;             % Element indexes
x           = n*p;                  % x-pos of each element
cordic_iter = 8;                    % Iterations used in CORDIC algorithm
delta_length= v/f_s;                % Increment length for scanline
num_points  = 2^12;                 % Number of points on scanline
scan_length = num_points*delta_length;     % Length of scanline
n0_index    = 32;                   % Array index of element in origo

% Variable input values, used as reference point in scanline
R_0 = 5*10^-3;%p*40;%50*p; 
angle_deg = 45;

% Calculating reference delays
angle = angle_deg*pi/180;
a = R_0*sin(angle);
b = x - R_0*cos(angle);
R_n = sqrt(a.^2+b.^2);
delay_reference = (f_s*R_n/v);

a1 = zeros(noElements);
b1 = zeros(noElements);
R_n1 = zeros(noElements);
delay_reference_scanline = zeros(num_points, noElements);

for k = 1:num_points
    a1 = (R_0+k*delta_length)*sin(angle);
    b1 = x - (R_0+k*delta_length)*cos(angle);
    R_n1 = sqrt(a1.^2+b1.^2);
    delay_reference_scanline(k,:) = (f_s*R_n1/v);
end


% SYSTEM FUNCTIONALITY %
% Step 1: Using iterative approach to calculate delays for each element for first point in scanline
[delay_sq,delay] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter);
% Step 2: Calculating delays in next point on scanline for all elements
[scanline_delays_sq, scanline_delays] = delay_scanline(R_0, f_s, v, n, p, num_points, delay_sq, cordic_iter, angle_deg);

% Plotting result with respect to reference %
plot_results(n, delay, angle, R_0, scanline_delays, delta_length, num_points, n0_index, delay_reference, x, delay_reference_scanline, 1);



% Functions

% CORDIC algorithm
function cos_a = cordic(iterations, angle_degrees, x_scale)
    % Initializing values
    K_n = 1; sign_bit = 1;
    if angle_degrees > 90
        angle_degrees = 180 - angle_degrees;
        sign_bit = -1; 
    end
    
    % Calculating K_n, this will be a constant precalculated in hardware memory
    for i = 0:iterations-1
        K_n = K_n * 1/sqrt(1+2^(-2*i));
    end
    x_0 = K_n*x_scale; y_0 = 0;
    B_0 = (angle_degrees/180)*pi;
    s = 1;

    % Performing iterative calculation
    for i = 0:iterations-1
        if i == 0
            B_i = B_0;
            x_i = x_0;
            y_i = y_0;
        else
            B_i = B_i1;
            x_i = x_i1;
            y_i = y_i1;
        end
    
        x_i1 = x_i - s * y_i * 2^(-i);
        y_i1 = s * x_i * 2^(-i) + y_i;
        B_i1 = B_i - s * atan(2^(-i));

        if B_i1 > 0
            s = 1;
        elseif B_i1 < 0
            s = -1;
        end
    end

    cos_a = sign_bit * x_i1;
end

% First scanpoint delays calculation
function [delay_sq_list, delay_list] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter)
    % Pre-calculating constants for each scanline, to increase throughput in hardware
    A_0 = (f_s*p/v)^2; 
    C_0 = cordic(cordic_iter, angle_deg, (f_s/v)^2 * 2 * p * R_0);
    
    % Calculating delay for reference transducer element in origo
    delay_sq(n0_index) = (f_s/v)^2*R_0^2;
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

    delay_sq_list = delay_sq;
    delay_list = delay;
end

% Next scanpoints delay calculation
function [delay_sq_array, delay_array] = delay_scanline(R_0, f_s, v, n, p, num_points,delay_sq, cordic_iter, angle_deg)
    B_n = 2 * R_0 * (f_s/v) - cordic(cordic_iter, angle_deg, 2 * n * p * (f_s/v));
    scanline_delays_sq = zeros(num_points, length(delay_sq));
    scanline_delays = zeros(num_points, length(delay_sq));
    
    scanline_delays_sq(1,:) = delay_sq;
    scanline_delays(1,:) = sqrt(delay_sq);
    for k = 2:num_points
        scanline_delays_sq(k,:) = scanline_delays_sq(k-1,:) + 2*k + 1 + B_n;
        scanline_delays(k,:) = sqrt(scanline_delays_sq(k,:));
    end

    delay_sq_array = scanline_delays_sq;
    delay_array = scanline_delays;
end

% Plotting function
function plot_results(n, delay, angle, R_0, scanline_delays, delta_length, num_points, n0_index, delay_reference, x, delay_reference_scanline, scanline_element_index)
    % Calculating difference between delay in origo and other elements
    for i = 1:length(n)
        delta_delay(i) = delay(i)-delay(n0_index);
    end
    
    % Calculating errors from reference values
    for i = 1:length(n)
        error_delay(i) = delay(i)-delay_reference(i);
    end
    
    % Plotting scanpoint
    figure(1);
    polarscatter(angle, R_0,'filled','blue'); hold on;
    polarplot([angle,angle], [0,R_0],'red'); hold on;
    title("Delay for each element to reference scanpoint");
    subtitle("Black indicates error when compared to theoretical reference");
    
    % Plotting illustration of delay for each element at first point of
    % scanline, error plotted in black
    delay_gain = R_0*0.01;
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
    hold off;
    
    % Plotting scanline delay for a given element
    figure(2);
    x_ax = R_0:delta_length:R_0+delta_length*(num_points-1);
    plot(x_ax,scanline_delays(:,scanline_element_index,:)); hold on
    plot(x_ax,delay_reference_scanline(:,scanline_element_index,:)); hold on
    xlabel("Distance travelled on scanline");
    ylabel("Delay in system clock cycles");
    legend("Iterative approach", "Theoretical reference");
    title(sprintf("Delay for element n=%d as focal points moves through scanline", scanline_element_index-32));
    hold off;

    % Plotting delta delay between each element as a function of
    % R_0+delta_length*k
    figure(3);
    for k = 1:1:num_points
        delta_scanline_delays = scanline_delays(k,:,1) - scanline_delays(k,32,1);
        
        plot(-32:1:31,delta_scanline_delays(:,:)); hold on;
    end
    title("Delta delay profile for elements");
    xlabel("Element index");
    ylabel("Delay in sample frequency cycles");
    subtitle(sprintf(" R_n = [%d mm,%.1d mm] and angle = %.0d", R_0*10^3, R_0*10^3 + 10^3*num_points * delta_length, angle*180/pi));
    hold off;
end
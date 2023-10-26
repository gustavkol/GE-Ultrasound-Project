% Static memory parameters %
noElements  = 64;                   % Number of transducer elements
p           = 250 * 10^-6;          % Pitch length
t_s         = 40*10^-9;             % Sample period
f_s         = 1/t_s;                % Sample frequency
f_clk       = 100 * 10^6;           % Clock frequency
v           = 1540;                 % Speed of sound
n           = -31:1:32;             % Element indexes
x           = n*p;                  % x-pos of each element
cordic_iter = 12;                   % Iterations used in CORDIC algorithm
delta_length= v/f_s;                % Increment length for scanline
num_points  = 2^12;                 % Number of points on scanline
scan_length = num_points*delta_length;% Length of scanline
n0_index    = 32;                   % Array index of element in origo
inc_step    = 1/4;                  % Increment step for a, inc_step*2 mirrors approximal maximal error

%Error is symmetric around 90 degrees -> only scanning one quadrant
max_error_neg = zeros(255); 
max_error_pos = zeros(255);
iter = 0;
max_iter_ref_point = zeros(255);
max_iter_ref_point_angle = zeros(45);
for R_0 = 1*10^-3:10^-3:255*10^-3
    iter = iter+1;
    scanlength = 255*10^-3 - R_0;
    num_points = round(scanlength/delta_length);
    for angle_deg = 90-45:90
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
            a1 = (R_0+(k-1)*delta_length)*sin(angle);
            b1 = x - (R_0+(k-1)*delta_length)*cos(angle);
            R_n1 = sqrt(a1.^2+b1.^2);
            delay_reference_scanline(k,:) = (f_s*R_n1/v);
        end


        % SYSTEM FUNCTIONALITY %
        % Step 1: Using iterative approach to calculate delays for each element for first point in scanline
        [delay,iter_count,a_list_ref,error_list_ref] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter, inc_step);
        if abs(iter_count) > max_iter_ref_point(iter)
            max_iter_ref_point(iter) = abs(iter_count);
        end
        if abs(iter_count) > max_iter_ref_point_angle(angle_deg-44)
            max_iter_ref_point_angle(angle_deg-44) = abs(iter_count);
        end
        

        disp("R_0: " + R_0 + ", angle: " + angle_deg)
        disp("Iterations (clock cycles) needed for reference point delay calculations: " + iter_count);
        % Step 2: Calculating delays in next point on scanline for all elements
        scanline_delays = delay_scanline(R_0, f_s, v, n, p, num_points, delay, cordic_iter, angle_deg, inc_step,a_list_ref, error_list_ref);

        % Sweeping scanline %
        [max_error_neg(iter), max_error_pos(iter)] = sweep_scanline(R_0, angle, scanline_delays, num_points, delay_reference_scanline, max_error_neg(iter), max_error_pos(iter), 0);
    end
    disp("R_0 = " + R_0);
    disp("Max negative error: " + max_error_neg(iter));
    disp("Max positive error: " + max_error_pos(iter));
    disp(" ")
end

figure(1);
plot(max_error_pos(1:254), 'g'); hold on; plot(abs(max_error_neg(1:254)),'r');
legend("Maximal positive error", "Maximal negative error");
ylabel("Error in sample frequency cycles/periods");
title("Maximal delay error with regards to initial scanpoint length R_0, step size = " + inc_step);
xlabel("R_0 (mm)");

% Plotting iterations needed for ref point as a function of R_0
figure(2)
plot(1:255,max_iter_ref_point(1:255))
title("Maximal number of iterations needed with regards to R_0, step size = " + inc_step);
ylabel("Iterations needed");
xlabel("R_0 (mm)");

% Plotting iterations needed for ref point as a function of R_0
figure(3)
plot(45:1:90,max_iter_ref_point_angle(1:46));
title("Maximal number of iterations needed with regards to angle, step size = " + inc_step);
ylabel("Iterations needed");
xlabel("angle (degrees)");
%% 

%Looking at some interesting R_0 values
figure(4);
R_0_list = [10^-3,2*10^-3, 129*10^-3, 242*10^-3];
max_error_angle = zeros(46);
for i = 1:length(R_0_list)
    iter = 0;
    R_0 = R_0_list(i);
    scanlength = 255*10^-3 - R_0;
    num_points = round(scanlength/delta_length);
    for angle_deg = 45:90
        iter = iter + 1;
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
            a1 = (R_0+(k-1)*delta_length)*sin(angle);
            b1 = x - (R_0+(k-1)*delta_length)*cos(angle);
            R_n1 = sqrt(a1.^2+b1.^2);
            delay_reference_scanline(k,:) = (f_s*R_n1/v);
        end


        % SYSTEM FUNCTIONALITY %
        % Step 1: Using iterative approach to calculate delays for each element for first point in scanline
        [delay,iter_count,a_list_ref,error_list_ref] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter, inc_step);
        disp("R_0: " + R_0 + ", angle: " + angle_deg)
        disp("Iterations (clock cycles) needed for reference point delay calculations: " + iter_count);
        % Step 2: Calculating delays in next point on scanline for all elements
        scanline_delays = delay_scanline(R_0, f_s, v, n, p, num_points, delay, cordic_iter, angle_deg, inc_step,a_list_ref, error_list_ref);

        % Sweeping scanline %
        [discard,max_error_angle(iter)] = sweep_scanline(R_0, angle, scanline_delays, num_points, delay_reference_scanline, 0, 0, 0);
        disp(angle_deg);
    end
    plot(45:90, max_error_angle(1:46)); hold on;
end
legend("R_0 = " + R_0_list(1),"R_0 = " + R_0_list(2), "R_0 = " + R_0_list(3), "R_0 = " + R_0_list(4));
ylabel("Error in sample frequency cycles/periods")
xlabel("Angle (degrees)")
title("Maximal positive delay error in scanline with regards to angle")
hold off;

%% 
R_0_list = [10^-3, 2*10^-3, 10*10^-3, 20*10^-3, 30*10^-3, 129*10^-3, 242*10^-3];
for i = 1:length(R_0_list)
    iter = 0;
    R_0 = R_0_list(i);
    scanlength = 255*10^-3 - R_0;
    num_points = round(scanlength/delta_length);
    
    angle_deg = 50;
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
        a1 = (R_0+(k-1)*delta_length)*sin(angle);
        b1 = x - (R_0+(k-1)*delta_length)*cos(angle);
        R_n1 = sqrt(a1.^2+b1.^2);
        delay_reference_scanline(k,:) = (f_s*R_n1/v);
    end
    
    
    % SYSTEM FUNCTIONALITY %
    % Step 1: Using iterative approach to calculate delays for each element for first point in scanline
    [delay,iter_count,a_list_ref,error_list_ref] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter, inc_step);
    disp("R_0: " + R_0 + ", angle: " + angle_deg)
    disp("Iterations (clock cycles) needed for reference point delay calculations: " + iter_count);
    % Step 2: Calculating delays in next point on scanline for all elements
    scanline_delays = delay_scanline(R_0, f_s, v, n, p, num_points, delay, cordic_iter, angle_deg, inc_step,a_list_ref, error_list_ref);
    
    % Sweeping scanline %
    figure(4+i);
    [discard,discard1] = sweep_scanline(R_0, angle, scanline_delays, num_points, delay_reference_scanline, 0, 0, 1);
end
%% 

% Functions

% First scanpoint delays calculation
function [delay_list,iter_count,a_list_ref,error_list_ref] = delay_ref_point(f_s, p, v, R_0, n0_index, angle_deg, cordic_iter, inc_step)
    % Pre-calculating constants for each scanline, to increase throughput in hardware
    A_0 = (f_s*p/v)^2; 
    C_0 = cordic(cordic_iter, angle_deg, (f_s/v)^2 * 2 * p * R_0);
    %C_0 = (f_s/v)^2 * 2 * p * R_0 * cosd(angle_deg);                % Theoretical value
    
    % Calculating delay for reference transducer element in origo
    delay(n0_index) = (f_s/v)*R_0;
    
    % Iteratively calculating delay for all elements for first scan point
    error_prev_list = zeros(64); a_prev_list = zeros(64); inc_term_prev = 0; iter_count_pos = 0;
    
    if angle_deg >= 120
        a_prev_list(n0_index) = 2;
    elseif angle_deg >= 105
        a_prev_list(n0_index) = 1;
    elseif angle_deg <= 55
        a_prev_list(n0_index) = -2;
    elseif angle_deg <= 75
        a_prev_list(n0_index) = -1;
    else
        a_prev_list(n0_index) = 0;
    end

    for i = 1:32
        cur_index = n0_index + i;
        inc_term = A_0*(2*(i-1)+1) - C_0;
        [delay(cur_index),error_prev_list(cur_index), a_prev_list(cur_index), inc_term_prev, iter_count_pos] = increment_and_compare(delay(cur_index-1), error_prev_list(cur_index-1), inc_term, a_prev_list(cur_index-1), inc_term_prev, inc_step, iter_count_pos);
    end
    inc_term_prev = 0; iter_count_neg = 0;

    if angle_deg >= 130
        a_prev_list(n0_index) = -2;
    elseif angle_deg >= 115
        a_prev_list(n0_index) = -1;
    elseif angle_deg <= 60
        a_prev_list(n0_index) = 2;
    elseif angle_deg <= 75
        a_prev_list(n0_index) = 1;
    else
        a_prev_list(n0_index) = 0;
    end

    for i = 1:31
        cur_index = n0_index - i;
        inc_term = A_0*(2*(i-1)+1) + C_0;
        [delay(cur_index),error_prev_list(cur_index), a_prev_list(cur_index), inc_term_prev, iter_count_neg] = increment_and_compare(delay(cur_index+1), error_prev_list(cur_index+1), inc_term, a_prev_list(cur_index+1), inc_term_prev, inc_step,iter_count_neg);
    end

    if iter_count_neg > iter_count_pos
        iter_count = iter_count_neg;
    else
        iter_count = iter_count_pos;
    end

    delay_list = delay;
    a_list_ref = a_prev_list;
    error_list_ref = error_prev_list;
end

% Increment and compare block for initial point on scanline (replaces square root)
function [N_next,error_next, a_next, inc_term_next, iter_count_out] = increment_and_compare(N_prev, error_prev, inc_term, a_prev, inc_term_prev, inc_step, iter_count_in)
    iter_count = iter_count_in;
    
    % Propagate previous a to get an initial guess for a
    a = a_prev;
    
    % Decide whether a has to increase or decrease
    inc_term_w_error = inc_term + error_prev;
    if inc_term_w_error >= inc_term_prev
        sign_bit = 1;
    else
        sign_bit = -1;
    end
    
    % Finding a through increment and comparator
    cur_error = 0;
    for i = 1:100
        iter_count = iter_count + 1;
        a = a + sign_bit*inc_step;          
        if sign_bit == -1
            if 2*a*N_prev+a^2 < inc_term_w_error
                a = a - sign_bit*inc_step;
                cur_error = inc_term_w_error - (2*a*N_prev + a^2);
                break;
            end
        elseif sign_bit == 1
            if 2*a*N_prev+a^2 > inc_term_w_error
                a = a - sign_bit*inc_step;
                cur_error = inc_term_w_error - (2*a*N_prev + a^2);
                break;
            end
        end
        %a = a + sign_bit*inc_step;
    end

    % Values to propagate to next calculation
    N_next = N_prev + a;
    error_next = cur_error;
    a_next = a;
    inc_term_next = 2*a*N_prev+a^2;
    iter_count_out = iter_count;
end

% Next scanpoints delay calculation
function delay_array = delay_scanline(R_0, f_s, v, n, p, num_points,delay, cordic_iter, angle_deg, inc_step, a_list_ref, error_list_ref)
    % Precalculated CORDIC constants, input except angle stored in memory
    B_cordic = cordic(cordic_iter, angle_deg, 2 * n * p * (f_s/v));
    
    % Precalculated constants
    B_n = 2 * R_0 * (f_s/v) - B_cordic;
    
    scanline_delays = zeros(num_points, length(delay));
    error_prev = error_list_ref;
    a_prev = zeros(length(delay));
    inc_term_prev = zeros(length(delay));
    
    scanline_delays(1,:) = delay;
    for k = 1:num_points-1
        inc_terms = 2*(k-1) + 1 + B_n;
        [scanline_delays(k+1,:),error_prev, a_prev, inc_term_prev] = increment_and_compare_array(scanline_delays(k,:), error_prev, inc_terms, a_prev, inc_term_prev, inc_step);
    end

    delay_array = scanline_delays;
end

% Increment and compare array block, for next point in scanline (replaces square root)
function [N_next_array,error_next_array, a_next_array, inc_term_next_array] = increment_and_compare_array(N_prev_array, error_prev_array, inc_terms_array, a_prev, inc_term_prev, inc_step)
    N_next_array_cur = zeros(length(N_prev_array));
    error_next_array_cur = zeros(length(N_prev_array));
    a_next_array_cur = zeros(length(N_prev_array));
    inc_term_next_array_cur = zeros(length(N_prev_array));

    for i = 1:length(N_prev_array)
        [N_next_array_cur(i),error_next_array_cur(i),a_next_array_cur(i), inc_term_next_array_cur(i), iter] = increment_and_compare(N_prev_array(i), error_prev_array(i), inc_terms_array(i), a_prev(i), inc_term_prev(i), inc_step, 0);
    end

    N_next_array = N_next_array_cur(1:length(N_prev_array));
    error_next_array = error_next_array_cur(1:length(N_prev_array));
    a_next_array = a_next_array_cur(1:length(N_prev_array));
    inc_term_next_array = inc_term_next_array_cur(1:length(N_prev_array));

end

% Sweeping scanline for specified angle
function [max_error_neg, max_error_pos] = sweep_scanline(R_0, angle, scanline_delays, num_points, delay_reference_scanline, max_error_neg_in, max_error_pos_in, plot_error)
    % Finding maximal error
    max_error_pos_cur = 0;
    max_error_neg_cur = 0;
    max_error_list = zeros(num_points);
    max_error_element_pos = 0;
    max_error_element_neg = 0;
    max_error_point_pos = 0;
    max_error_point_neg = 0;
    for k = 1:num_points
        for n = 1:64
            if abs(scanline_delays(k,n) - delay_reference_scanline(k,n,:)) > abs(max_error_list(k))
                max_error_list(k) = scanline_delays(k,n) - delay_reference_scanline(k,n);
            end
            if scanline_delays(k,n) - delay_reference_scanline(k,n,:) > max_error_pos_cur
                max_error_pos_cur = scanline_delays(k,n) - delay_reference_scanline(k,n);
                max_error_element_pos = n - 32;
                max_error_point_pos = k;
            end
            if scanline_delays(k,n) - delay_reference_scanline(k,n,:) < max_error_neg_cur
                max_error_neg_cur = scanline_delays(k,n) - delay_reference_scanline(k,n);
                max_error_element_neg = n - 32;
                max_error_point_neg = k;
            end
        end
    end
    if max_error_pos_cur > max_error_pos_in
        max_error_pos = max_error_pos_cur;
    else
        max_error_pos = max_error_pos_in;
    end

    if max_error_neg_cur < max_error_neg_in
        max_error_neg = max_error_neg_cur;
    else
        max_error_neg = max_error_neg_in;
    end

    if plot_error == 1
        plot(1:num_points,max_error_list(1:num_points),'r')
        legend("R_0 = " + R_0 + ", angle = " + 180*angle/pi);
        ylabel("Error in sample frequency cycles/periods")
        xlabel("Point on scanline (k)")
        title("Maximal delay error with regards to each point k on scanline")
        hold off;

        disp("angle = " + 180*angle/pi + ": Maximal pos error in scanline: " + max_error_pos + " (element " + max_error_element_pos + " point " + max_error_point_pos + ")");
        disp("angle = " + 180*angle/pi + ": Maximal neg error in scanline: " + max_error_neg + " (element " + max_error_element_neg + " point " + max_error_point_neg + ")");
        disp(" ");
    end
end
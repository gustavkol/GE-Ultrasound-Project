n = 10; % Number of iterations
initAngle = 45; % Initial angle
lastAngle = 90; % Last angle

for angle = initAngle : lastAngle
    % Initialization values
    B_0 = (angle/180)*pi;
    x_0 = 1; y_0 = 0;                      % Init xy-values
    s = 1;
     
    % Variables
    x_i = 0; x_i1 = 0;
    y_i = 0; y_i1 = 0;
    B_i = 0; B_i1 = 0;
    K = 0;

    % Calculating precalculated constant K for n iterations
    for i = 0:n-1
        if i == 0
            K = 1/sqrt(1+2^(-2*i));
        else
            K = K * 1/sqrt(1+2^(-2*i));
        end
    end
    
    for i = 0:n-1
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
    
    % Multiplying with precalculated constant K
    cosine_theta =  K * x_i1;
    sine_theta = K * y_i1;
    
    diffCosine = cos(B_0) - cosine_theta;
    diffSine = sin(B_0) - sine_theta;
    disp("Angle: " + 180*B_0/pi);
    disp("Cosine: " + cos(B_0) + ", Calculated: " + cosine_theta + ", Difference: " + diffCosine);
    disp("Sine: " + sin(B_0) + ", Calculated: " + sine_theta + ", Difference: " + diffSine);
    fprintf("\n");

    B_0 = B_0 + pi/180;
end

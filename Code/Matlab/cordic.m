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
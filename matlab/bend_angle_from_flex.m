function [q] = bend_angle_from_flex(flex_vec)
    % bend_angle_from_right_flex uses the parameters from the ipynb to find the 
    % bend angle from the right fliex data
    m = 349469.80611246;
    b = -1.00449021;

    q = m * (flex_vec + 1e-6).^(-2) + b;
end






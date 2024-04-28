function h_sn = my_h_sn(s_hat_n_given_n_1)
    h_sn = zeros(2, 1);
    if s_hat_n_given_n_1(1,1) <=0
        angle = (pi + atan(s_hat_n_given_n_1(2,1)/s_hat_n_given_n_1(1,1)));
    else
        angle = atan(s_hat_n_given_n_1(2,1)/s_hat_n_given_n_1(1,1));
    end
    h_sn(1,1) = sqrt(s_hat_n_given_n_1(1,1)^2 + s_hat_n_given_n_1(2,1)^2);
    h_sn(2,1) = angle;
   
end
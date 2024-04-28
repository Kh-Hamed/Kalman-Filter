function H_n = my_H_n(s_hat_n_given_n_1)
    H_n = zeros(2, 4);
    H_n(1,1) = s_hat_n_given_n_1(1,1)/sqrt(s_hat_n_given_n_1(1,1)^2 + s_hat_n_given_n_1(2,1)^2);
    H_n(1,2) = s_hat_n_given_n_1(2,1)/sqrt(s_hat_n_given_n_1(1,1)^2 + s_hat_n_given_n_1(2,1)^2);
    H_n(2,1) = -s_hat_n_given_n_1(2,1)/(s_hat_n_given_n_1(1,1)^2 + s_hat_n_given_n_1(2,1)^2);
    H_n(2,2) = s_hat_n_given_n_1(1,1)/(s_hat_n_given_n_1(1,1)^2 + s_hat_n_given_n_1(2,1)^2);
   
end
function [U] = input_dynamics(d_f, F_x, U_last, N, scale)
d_f_rate = 0.005;
F_x_rate = 100;

% Don't allow input out of range
if(d_f >= 0.5)
    d_f = 0.5;
end
if(d_f <= -0.5)
    d_f = -0.5;
end
if(F_x >= 5000)
    F_x = 5000;
end
if(F_x <= -10000)
    F_x = -10000;
end



% Don't allow input to change instantaneously
U = zeros(N,2);

% d_f dynamics at most full left to full right in 1 sec (0.01 per iter)
d_f_last = U_last(end,1);
d_f_error = d_f_last - d_f;

if( abs(d_f_error) <= d_f_rate)
   U(:,1) = d_f * ones(N,1); 
else
    if (d_f_error > 0)
        d_f_step = -1*d_f_rate;
    else
        d_f_step = d_f_rate;
    end
    for i=1:scale:N
        U(i:(i+scale-1),1) = (d_f_last + d_f_step) * ones(scale,1);
        d_f_last = U(i,1);
        d_f_error = d_f_error + d_f_step;
        if(abs(d_f_error) <= d_f_rate)
            U(i+scale:end,1) = d_f *ones(N-(i+scale)+1,1);
            break;
        end
    end
end

% F_x dynamics at most 10,000 per 0.1 sec == 1000 per second
F_x_last = U_last(end,2);
F_x_error = F_x_last - F_x;

if( abs(F_x_error) <= F_x_rate)
   U(:,2) = F_x * ones(N,1); 
else
    if (F_x_error > 0)
        F_x_step = -1*F_x_rate;
    else
        F_x_step = F_x_rate;
    end    
    for i=1:scale:N
        U(i:(i+scale-1),2) = (F_x_last + F_x_step) * ones(scale,1);
        F_x_last = U(i,2);
        F_x_error = F_x_error + F_x_step;
        if(abs(F_x_error) <= F_x_rate)
            U(i+scale:end,2) = F_x * ones(N-(i+scale)+1,1);
            break;
        end
    end
end


end


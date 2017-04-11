function T_out = g_inv(T_in)

    R = T_in(1:3,1:3);
    P = T_in(1:3,4);
    T_out=[transpose(R) -transpose(R)*P;
            0 0 0 1];
end

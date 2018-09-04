%% Test 360 Input
F_x = 0;
d_f = 0;
for i=1:1000
    [d_f, F_x, Esc] = get_360_input();
    d_f
    F_x
    Esc
    pause(0.1);
end
function a = smothstep(Prv_val,New_val,Step_time)
T = Step_time;

A = [1 0 0 0 0;1 T/2 T^2/4 T^3/8 T^4/16;1 T T^2 T^3 T^4;0 1 0 0 0;0 1 2*T 3*T^2 4*T^3];
X0 = [Prv_val;(Prv_val+New_val)/2;New_val;0;0];
a = A^-1*X0;

% x = a(1) + a(2).*t + a(3) .* t.^2 + a(4) .* t.^3 + a(5) .* t.^4;

end
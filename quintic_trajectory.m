function q = quintic_trajectory(tstart, tend, qstart, qend, qdotstart, qdotend)
% Quintic interpolation.

% given q=a0 + a1t + a2t^2 + a3t^3 + a4t^4+a5t^5, the coefficients are 
%solved for by applying the four boundary conditions

A=[ 1 tstart tstart^2 tstart^3 tstart^4 tstart^5; 
    0 1 2*tstart 3*tstart^2 4*tstart^3  5*tstart^4;
    0 0 2        6*tstart   12*tstart^2 20*tstart^3;
    1 tend       tend^2     tend^3 tend^4 tend^5;
    0 1 2*tend 3*tend^2     4*tend^3    5*tend^4;
    0 0 2      6*tend   12*tend^2       20*tend^3];

%acceleration at the strat and end is set to 0
B=[ qstart; qdotstart;0; qend; qdotend;0];

a=A\B;

q=a;%(1)+a(2)*t+a(3)*t^2+a(4)*t^3+a(5)*t^4+a(6)*t^5;

end


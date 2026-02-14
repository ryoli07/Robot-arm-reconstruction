function [e,c] = Input(t,p)
    theta_test = t;
    phi_test = p;
    
    e = [0 -9.8 0];
    R_e = [cos(theta_test) -sin(theta_test) 0; sin(theta_test) cos(theta_test) 0; 0 0 1];
    R_b = [1 0 0; 0 cos(phi_test) -sin(phi_test); 0 sin(phi_test) cos(phi_test)];
    
    c = e * R_e * R_b;
    
    e = e';
    c = c';
end

function [Lt,Lp,Ltt,Ltp,Lpt,Lpp] = Diff(theta,phi,t,p) 
    [e_vector,c_vector] = Input(t,p);

    Rt = [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0; 0 0 1];
    Rp = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    R1t = [-sin(theta) cos(theta) 0; -cos(theta) -sin(theta) 0; 0 0 0];
    R1p = [0 0 0; 0 -sin(phi) cos(phi); 0 -cos(phi) -sin(phi)];
    R2t = [-cos(theta) -sin(theta) 0; sin(theta) -cos(theta) 0; 0 0 0];
    R2p = [0 0 0; 0 -cos(phi) -sin(phi); 0 sin(phi) -cos(phi)];
    
    G = Rp * Rt * e_vector - c_vector;
    Gt = Rp * R1t * e_vector;
    Gp = R1p * Rt * e_vector;
    Gtt = Rp * R2t * e_vector;
    Gtp = R1p * R1t * e_vector;
    Gpp = R2p * Rt * e_vector;
    Gpt = R1p * R1t * e_vector;
    
    Lt = 2 * G' * Gt;
    Lp = 2 * G' * Gp;
    Ltt = 2 * (Gt)' * Gt + 2 * G' * Gtt;
    Ltp = 2 * (Gp)' * Gt + 2 * G' * Gtp;
    Lpt = 2 * (Gt)' * Gp + 2 * G' * Gpt;
    Lpp = 2 * (Gp)' * Gp + 2 * G' * Gpp;
end

theta_test = pi/4;
phi_test = pi/4;

theta = 0.3;
phi = 0.3;

while true
    [Lt,Lp,Ltt,Ltp,Lpt,Lpp] = Diff(theta,phi,theta_test,phi_test);

    R_newton = [Ltt,Ltp;Lpt,Lpp];
    R_newton_inv = inv(R_newton);


    newton = [theta;phi] - R_newton_inv * [Lt;Lp];
    theta2 = newton(1,1);
    phi2 = newton(2,1);

    if abs(theta2 - theta) < 0.0001 && abs(phi2 - phi) < 0.0001
        break
    end

    theta = theta2;
    phi = phi2;
end

disp("theta: "+theta)
disp("theta_test: "+theta_test);
disp("----------------")
disp("phi: "+phi )
disp("phi_test: "+phi_test);

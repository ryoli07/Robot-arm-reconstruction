 function [R,R_d1,R_d2] = Rotate(angle,axis)
    Cos  = cos(angle);
    Sin = sin(angle);
    
    Rx = [1 0 0; 0 Cos Sin; 0 -Sin Cos];
    Ry = [Cos 0 -Sin; 0 1 0; Sin 0 Cos];
    Rz = [Cos Sin 0; -Sin Cos 0; 0 0 1];
    Rx_d1 = [0 0 0; 0 -Sin Cos; 0 -Cos -Sin];
    Ry_d1 = [-Sin 0 -Cos; 0 0 0; Cos 0 -Sin];
    Rz_d1 = [-Sin Cos 0; -Cos -Sin 0; 0 0 0];
    Rx_d2 = [0 0 0; 0 -Cos -Sin; 0 Sin -Cos];
    Ry_d2 = [-Cos 0 Sin; 0 0 0; -Sin 0 -Cos];
    Rz_d2 = [-Cos -Sin 0; Sin -Cos 0; 0 0 0];
    
    if axis == 0
        R = Rx;
        R_d1 = Rx_d1;
        R_d2 = Rx_d2;
    elseif axis == 1
        R = Ry;
        R_d1 = Ry_d1;
        R_d2 = Ry_d2;
    elseif axis == 2
        R = Rz;
        R_d1 = Rz_d1;
        R_d2 = Rz_d2;
    end
end

function[c] = Test(angle_1test,angle_2test,angle_3test,axis_1,axis_2,axis_3,e_vector)
    R1_test = Rotate(angle_1test,axis_1);
    R2_test = Rotate(angle_2test,axis_2);
    R3_test = Rotate(angle_3test,axis_3);
    c = e_vector' * R1_test' * R2_test' * R3_test';
    c = c';
end  
   

function [L1,L2,L3,L11,L12,L13,L21,L22,L23,L31,L32,L33] = Diff(angle_1,angle_2,angle_3,axis_1,axis_2,axis_3,e_vector,c_vector)
    [R1,R1_d1,R1_d2] = Rotate(angle_1,axis_1);
    [R2,R2_d1,R2_d2] = Rotate(angle_2,axis_2);
    [R3,R3_d1,R3_d2] = Rotate(angle_3,axis_3);
    
    G = R3 * R2 * R1 * e_vector - c_vector;
    G1 = R3 * R2 * R1_d1 * e_vector;
    G2 = R3 * R2_d1 * R1 * e_vector;
    G3 = R3_d1 * R2 * R1 * e_vector;
    G11 = R3 * R2 * R1_d2 * e_vector;
    G12 = R3 * R2_d1 * R1_d1 * e_vector;
    G13 = R3_d1 * R2 * R1_d1 * e_vector;
    G21 = R3 * R2_d1 * R1_d1 * e_vector;
    G22 = R3 * R2_d2 * R1 * e_vector;
    G23 = R3_d1 * R2_d1 * R1 * e_vector;
    G31 = R3_d1 * R2 * R1_d1 * e_vector;
    G32 = R3_d1 * R2_d1 * R1 * e_vector;
    G33 = R3_d2 * R2 * R1 * e_vector;
    
    G_T = G';
    G1_T  = G1';
    G2_T = G2';
    G3_T = G3';
    
    L1 = 2 * G_T * G1;
    L2 = 2 * G_T * G2;
    L3 = 2 * G_T * G3;
    L11 = 2 * G1_T * G1 + 2 * G_T * G11;
    L12 = 2 * G2_T * G1 + 2 * G_T * G12;
    L13 = 2 * G3_T * G1 + 2 * G_T * G13;
    L21 = 2 * G1_T * G2 + 2 * G_T * G21;
    L22 = 2 * G2_T * G2 + 2 * G_T * G22;
    L23 = 2 * G3_T * G2 + 2 * G_T * G23;
    L31 = 2 * G1_T * G3 + 2 * G_T * G31;
    L32 = 2 * G2_T * G3 + 2 * G_T * G32;
    L33 = 2 * G3_T * G3 + 2 * G_T * G33;
end

%コードスタート
angle_1 = 0.8;
angle_2 = 0.1;
angle_3 = 0.4;
axis_1 = 0;
axis_2 = 1;
axis_3 = 2;
angle_1test = 0.7;
angle_2test = 0.7;
angle_3test = 0.7;
e_vector = [0;-1;0];
[c_vector] = Test(angle_1test,angle_2test,angle_3test,axis_1,axis_2,axis_3,e_vector);

figure;
hold on;
xlabel('Count');
ylabel('Loss');
grid on;
ylim([-1,1]);
xlim([0,5]);


count = 0;
tic
ca = [];
L1a = [];
L2a = [];
L3a = [];
while true
     [L1,L2,L3,L11,L12,L13,L21,L22,L23,L31,L32,L33] = Diff(angle_1,angle_2,angle_3,axis_1,axis_2,axis_3,e_vector,c_vector);
     ca =[ca count];
     L1a = [L1a L1];
     L2a = [L2a L2];
     L3a = [L3a L3];
     plot(ca,L1a);
     drawnow;
     plot(ca, L2a)
     plot(ca, L3a)
    R_newton = [L11,L12,L13; L21,L22,L23; L31,L32,L33];
    % R_newton_inv = inv(R_newton);
    newton = [angle_1;angle_2;angle_3] - R_newton \ [L1;L2;L3];
    angle_1_new = newton(1,1);
    angle_2_new = newton(2,1);
    angle_3_new = newton(3,1);

    if abs(angle_1_new - angle_1) < 0.0001 && abs(angle_2_new - angle_2) < 0.0001 && abs(angle_3_new - angle_3) < 0.0001
        break
    end

    angle_1 = angle_1_new;
    angle_2 = angle_2_new;
    angle_3 = angle_3_new;
    count = count + 1;
end
toc

[v_vector] = Test(angle_1,angle_2,angle_3,axis_1,axis_2,axis_3,e_vector);



c_vector_x = c_vector(1,1);
c_vector_y = c_vector(2,1);
c_vector_z = c_vector(3,1);
v_vector_x = v_vector(1,1);
v_vector_y = v_vector(2,1);
v_vector_z = v_vector(3,1);


fprintf("result/true x:%f/%f y:%f/%f z:%f/%f count:%d",v_vector_x,c_vector_x,v_vector_y,c_vector_y,v_vector_z,c_vector_z,count);
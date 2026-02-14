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

function[c] = Test(angle_1test,angle_2test,angle_3test,angle_4test,angle_5test,axis_1,axis_2,axis_3,axis_4,axis_5,e_vector)
    R1_test = Rotate(angle_1test,axis_1);
    R2_test = Rotate(angle_2test,axis_2);
    R3_test = Rotate(angle_3test,axis_3);
    R4_test = Rotate(angle_4test,axis_4);
    R5_test = Rotate(angle_5test,axis_5);
    c = e_vector' * R1_test' * R2_test' * R3_test' * R4_test' * R5_test';
    c = c';
end

function [L1,L2,L3,L4,L5,L11,L12,L13,L14,L15,L21,L22,L23,L24,L25,L31,L32,L33,L34,L35,L41,L42,L43,L44,L45,L51,L52,L53,L54,L55] = Diff(angle_1,angle_2,angle_3,angle_4,angle_5,axis_1,axis_2,axis_3,axis_4,axis_5,e_vector,c_vector)
    [R1,R1_d1,R1_d2] = Rotate(angle_1,axis_1);
    [R2,R2_d1,R2_d2] = Rotate(angle_2,axis_2);
    [R3,R3_d1,R3_d2] = Rotate(angle_3,axis_3);
    [R4,R4_d1,R4_d2] = Rotate(angle_4,axis_4);
    [R5,R5_d1,R5_d2] = Rotate(angle_5,axis_5);
    
    
    G = R5 * R4 * R3 * R2 * R1 * e_vector - c_vector;
    G1 = R5 * R4 * R3 * R2 * R1_d1 * e_vector;
    G2 = R5 * R4 * R3 * R2_d1 * R1 * e_vector;
    G3 = R5 * R4 * R3_d1 * R2 * R1 * e_vector;
    G4 = R5 * R4_d1 * R3 * R2 * R1 * e_vector;
    G5 = R5_d1 * R4 * R3 * R2 * R1 * e_vector;
    G11 = R5 * R4 * R3 * R2 * R1_d2 * e_vector;
    G12 = R5 * R4 * R3 * R2_d1 * R1_d1 * e_vector;
    G13 = R5 * R4 * R3_d1 * R2 * R1_d1 * e_vector;
    G14 = R5 * R4_d1 * R3 * R2 * R1_d1 * e_vector;
    G15 = R5_d1 * R4 * R3 * R2 * R1_d1 * e_vector;
    G22 = R5 * R4 * R3 * R2_d2 * R1 * e_vector;
    G23 = R5 * R4 * R3_d1 * R2_d1 * R1 * e_vector;
    G24 = R5 * R4_d1 * R3 * R2_d1 * R1 * e_vector;
    G25 = R5_d1 * R4 * R3 * R2_d1 * R1 * e_vector;
    G33 = R5 * R4 * R3_d2 * R2 * R1 * e_vector;
    G34 = R5 * R4_d1 * R3_d1 * R2 * R1 * e_vector;
    G35 = R5_d1 * R4 * R3_d1 * R2 * R1 * e_vector;
    G44 = R5 * R4_d2 * R3 * R2 * R1 * e_vector;
    G45 = R5_d1 * R4_d1 * R3 * R2 * R1 * e_vector;
    G55 = R5_d2 * R4 * R3 * R2 * R1 * e_vector;

    G_T = G';
    G1_T = G1';
    G2_T = G2';
    G3_T = G3';
    G4_T = G4';
    G5_T = G5';
    
    L1 = 2 * G_T * G1;
    L2 = 2 * G_T * G2;
    L3 = 2 * G_T * G3;
    L4 = 2 * G_T * G4;
    L5 = 2 * G_T * G5;
    L11 = 2 * G1_T * G1 + 2 * G_T * G11;
    L12 = 2 * G2_T * G1 + 2 * G_T * G12;
    L13 = 2 * G3_T * G1 + 2 * G_T * G13;
    L14 = 2 * G4_T * G1 + 2 * G_T * G14;
    L15 = 2 * G5_T * G1 + 2 * G_T * G15;
    L21 = 2 * G1_T * G2 + 2 * G_T * G12;
    L22 = 2 * G2_T * G2 + 2 * G_T * G22;
    L23 = 2 * G3_T * G2 + 2 * G_T * G23;
    L24 = 2 * G4_T * G2 + 2 * G_T * G24;
    L25 = 2 * G5_T * G2 + 2 * G_T * G25;
    L31 = 2 * G1_T * G3 + 2 * G_T * G13;
    L32 = 2 * G2_T * G3 + 2 * G_T * G23;
    L33 = 2 * G3_T * G3 + 2 * G_T * G33;
    L34 = 2 * G4_T * G3 + 2 * G_T * G34;
    L35 = 2 * G5_T * G3 + 2 * G_T * G35;
    L41 = 2 * G1_T * G3 + 2 * G_T * G14;
    L42 = 2 * G2_T * G3 + 2 * G_T * G24;
    L43 = 2 * G3_T * G3 + 2 * G_T * G34;
    L44 = 2 * G4_T * G3 + 2 * G_T * G44;
    L45 = 2 * G5_T * G3 + 2 * G_T * G45;
    L51 = 2 * G1_T * G3 + 2 * G_T * G15;
    L52 = 2 * G2_T * G3 + 2 * G_T * G25;
    L53 = 2 * G3_T * G3 + 2 * G_T * G35;
    L54 = 2 * G4_T * G3 + 2 * G_T * G45;
    L55 = 2 * G5_T * G3 + 2 * G_T * G55;
end

%コードスタート
angle_1 = 0.72;
angle_2 = 0.73;
angle_3 = 0.72;
angle_4 = 0.52;
angle_5 = 0.32;
axis_1 = 0;
axis_2 = 1;
axis_3 = 2;
axis_4 = 0;
axis_5 = 1;
angle_1test = 0.7;
angle_2test = 0.7;
angle_3test = 0.7;
angle_4test = 0.5;
angle_5test = 0.3;
e_vector = [0;-1;0];
[c_vector] = Test(angle_1test,angle_2test,angle_3test,angle_4test,angle_5test,axis_1,axis_2,axis_3,axis_4,axis_5,e_vector);
count = 0;

% figure;
% hold on;
% xlabel('Count');
% ylabel('Loss');
% grid on;
% ylim([-1,1]);
% xlim([0,30]);
% 
% ca = [];
% L1a = [];
% L2a = [];
% L3a = [];
% L4a = [];
% L5a = [];

tic
while true
     [L1,L2,L3,L4,L5,L11,L12,L13,L14,L15,L21,L22,L23,L24,L25,L31,L32,L33,L34,L35,L41,L42,L43,L44,L45,L51,L52,L53,L54,L55] = Diff(angle_1,angle_2,angle_3,angle_4,angle_5,axis_1,axis_2,axis_3,axis_4,axis_5,e_vector,c_vector);

     % ca =[ca count];
     % L1a = [L1a L1];
     % L2a = [L2a L2];
     % L3a = [L3a L3];
     % L4a = [L4a L4];
     % L5a = [L5a L5];
     % plot(ca,L1a);
     % plot(ca, L2a);
     % plot(ca, L3a);
     % plot(ca, L4a);
     % plot(ca, L5a);





    R_newton = [L11,L12,L13,L14,L15; L21,L22,L23,L24,L25; L31,L32,L33,L34,L35; L41,L42,L43,L44,L45; L51,L52,L53,L54,L55];
    % R_newton_inv = inv(R_newton);
    newton = [angle_1;angle_2;angle_3;angle_4;angle_5] - R_newton \ [L1;L2;L3;L4;L5];
    angle_1_new = newton(1,1);
    angle_2_new = newton(2,1);
    angle_3_new = newton(3,1);
    angle_4_new = newton(4,1);
    angle_5_new = newton(5,1);

    if abs(angle_1_new - angle_1) < 0.0001 && abs(angle_2_new - angle_2) < 0.0001 && abs(angle_3_new - angle_3) < 0.0001 && abs(angle_4_new - angle_4) < 0.0001 && abs(angle_5_new - angle_5) < 0.0001
        break
    end

    angle_1 = angle_1_new;
    angle_2 = angle_2_new;
    angle_3 = angle_3_new;
    angle_4 = angle_4_new;
    angle_5 = angle_5_new;
    count = count + 1;
end
toc
[v_vector] = Test(angle_1,angle_2,angle_3,angle_4,angle_5,axis_1,axis_2,axis_3,axis_4,axis_5,e_vector);
c_vector_x = c_vector(1,1);
c_vector_y = c_vector(2,1);
c_vector_z = c_vector(3,1);
v_vector_x = v_vector(1,1);
v_vector_y = v_vector(2,1);
v_vector_z = v_vector(3,1);


fprintf("result/true x:%f/%f y:%f/%f z:%f/%f count:%d",v_vector_x,c_vector_x,v_vector_y,c_vector_y,v_vector_z,c_vector_z,count);
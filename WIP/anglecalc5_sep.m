% 回転行列計算
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

%2軸の加速度計算
function[acc] = Calc2axis(angle1,angle2,axis1,axis2,base_vector)
    R1 = Rotate(angle1,axis1);
    R2 = Rotate(angle2,axis2);
    acc = R2 * R1 * base_vector;
end  

%3軸の加速度計算
function[acc] = Calc5axis(angle1,angle2,angle3,angle4,angle5,axis1,axis2,axis3,axis4,axis5,base_vector)
    R1 = Rotate(angle1,axis1);
    R2 = Rotate(angle2,axis2);
    R3 = Rotate(angle3,axis3);
    R4 = Rotate(angle4,axis4);
    R5 = Rotate(angle5,axis5);
    acc = R5 * R4 * R3 * R2 * R1 * base_vector;
end

%微分計算
function [L1,L2,L3,L4,L5,L11,L12,L21,L22,L33,L34,L35,L43,L44,L45,L53,L54,L55] = Diff(angle1,angle2,angle3,angle4,angle5,axis1,axis2,axis3,axis4,axis5,base_vector,acc_1st,acc_2nd)
    [R1,R1_d1,R1_d2] = Rotate(angle1,axis1);
    [R2,R2_d1,R2_d2] = Rotate(angle2,axis2);
    [R3,R3_d1,R3_d2] = Rotate(angle3,axis3);
    [R4,R4_d1,R4_d2] = Rotate(angle4,axis4);
    [R5,R5_d1,R5_d2] = Rotate(angle5,axis5);

    Err_1st = R2 * R1 * base_vector - acc_1st;
    Err_2nd = R5 * R4 * R3 * acc_1st - acc_2nd;

    E1_p1 = R2 * R1_d1 * base_vector;
    E1_p2 = R2_d1 * R2 * base_vector;
    E1_p11 = R2 * R1_d2 * base_vector;
    E1_p12 = R2_d1 * R1_d1 * base_vector;
    E1_p22 = R2_d2 * R1 * base_vector;

    E2_p3 = R5 * R4 * R3_d1 * acc_1st;
    E2_p4 = R5 * R4_d1 * R3* acc_1st;
    E2_p5 = R5_d1 * R4 * R3 * acc_1st;
    E2_p33 = R5 * R4 * R3_d2 * acc_1st;
    E2_p34 = R5 * R4_d1 * R3_d1 * acc_1st;
    E2_P35 = R5_d1 * R4 * R3_d1 * acc_1st;
    E2_p44 = R5 * R4_d2 * R3 * acc_1st;
    E2_p45 = R5_d1 * R4_d1 * R3 * acc_1st;
    E2_P55 = R5_d2 * R4 * R3 * acc_1st;

    E2_T = Err_2nd';
    E2_p3_T = E2_p3';
    E2_p4_T = E2_p4';
    E2_p5_T = E2_p5';

    E1_T = Err_1st';
    E1_p1_T = E1_p1';
    E1_p2_T = E1_p2';

    L1 = 2 * E1_T * E1_p1;
    L2 = 2 * E1_T * E1_p2;
    L3 = 2 * E2_T * E2_p3;
    L4 = 2 * E2_T * E2_p4;
    L5 = 2 * E2_T * E2_p5;

    L11 = 2 * E1_p1_T * E1_p1 + 2 * E1_T * E1_p11;
    L12 = 2 * E1_p2_T * E1_p1 + 2 * E1_T * E1_p12;
    L21 = 2 * E1_p1_T * E1_p2 + 2 * E1_T * E1_p12;
    L22 = 2 * E1_p2_T * E1_p2 + 2 * E1_T * E1_p22;
    L33 = 2 * E2_p3_T * E2_p3 + 2 * E2_T * E2_p33;
    L34 = 2 * E2_p4_T * E2_p3 + 2 * E2_T * E2_p34;
    L35 = 2 * E2_p5_T * E2_p3 + 2 * E2_T * E2_P35;
    L43 = 2 * E2_p3_T * E2_p4 + 2 * E2_T * E2_p34;
    L44 = 2 * E2_p4_T * E2_p4 + 2 * E2_T * E2_p44;
    L45 = 2 * E2_p5_T * E2_p4 + 2 * E2_T * E2_p45;
    L53 = 2 * E2_p3_T * E2_p5 + 2 * E2_T * E2_P35;
    L54 = 2 * E2_p4_T * E2_p5 + 2 * E2_T * E2_p45;
    L55 = 2 * E2_p5_T * E2_p5 + 2 * E2_T * E2_P55;
end

% ---------------------------コードスタート---------------------------------------
base_vector = [0;-1;0];

% 角度の初期値
angle1_init = 0.63;
angle2_init = 0.64;
angle3_init = 0.82;
angle4_init = 0.73;
angle5_init = 0.73;

% 回転軸指定 (0=x, 1=y, 2=z)
axis1 = 0;
axis2 = 1;
axis3 = 2;
axis4 = 0;
axis5 = 1;

% 取得データ計算
angle1_calc = 0.6;
angle2_calc = 0.6;
angle3_calc = 0.8;
angle4_calc = 0.7;
angle5_calc = 0.7;
[acc_1st] = Calc2axis(angle1_calc,angle2_calc,axis1,axis2,base_vector);
[acc_2nd] =Calc5axis(angle1_calc,angle2_calc,angle3_calc,angle4_calc,angle5_calc,axis1,axis2,axis3,axis4,axis5,base_vector);

%試行回数
count = 0;

%グラフ作図
% figure;
% hold on;
% xlabel('Count');
% ylabel('Loss');
% grid on; 
% ylim([-1,1]);
% xlim([0,7]);
% ca = [];
% L1a = [];
% L2a = [];
% L3a = [];
% L4a = [];
% L5a = [];


tic
while true
    [L1,L2,L3,L4,L5,L11,L12,L21,L22,L33,L34,L35,L43,L44,L45,L53,L54,L55] = Diff(angle1_init,angle2_init,angle3_init,angle4_init,angle5_init,axis1,axis2,axis3,axis4,axis5,base_vector,acc_1st,acc_2nd);
    
    % ca =[ca count];
    % L1a = [L1a L1];
    % L2a = [L2a L2];
    % L3a = [L3a L3];
    % L4a = [L4a L4];
    % L5a = [L5a L5];

    R_newton2 = [L11,L12;L21,L22];
    R_newton3 = [L33,L34,L35;L43,L44,L45;L53,L54,L55];
    
    newton2 = [angle1_init;angle2_init] - R_newton2 \[L1;L2];
    newton3 = [angle3_init;angle4_init;angle5_init] - R_newton3 \[L3;L4;L5];

    angle1_new = newton2(1,1);
    angle2_new = newton2(2,1);
    angle3_new = newton3(1,1);
    angle4_new = newton3(2,1);
    angle5_new = newton3(3,1);

    if abs(angle1_new - angle1_init) < 0.0001 && abs(angle2_new - angle2_init) < 0.0001 && abs(angle3_new - angle3_init) < 0.0001 && abs(angle4_new - angle4_init) < 0.0001 && abs(angle5_new - angle5_init) < 0.0001
        break
    end

    angle1_init = angle1_new;
    angle2_init = angle2_new;
    angle3_init = angle3_new;
    angle4_init = angle4_new;
    angle5_init = angle5_new;
    count = count + 1;
end
toc

 % plot(ca,L1a);
 % plot(ca, L2a);
 % plot(ca, L3a);
 % plot(ca, L4a);
 % plot(ca, L5a);
loss1 = (angle1_calc - angle1_init) / angle1_calc;
loss2 = (angle2_calc - angle2_init) / angle2_calc;
loss3 = (angle3_calc - angle3_init) / angle3_calc;
loss4 = (angle4_calc - angle4_init) / angle4_calc;
loss5 = (angle5_calc - angle5_init) / angle5_calc;

fprintf("Result/True Angle1:%f/%f,Angle2:%f/%f,Angle3:%f/%f,Angle4:%f/%f,Angle5:%f/%f,count:%d\n",angle1_init,angle1_calc,angle2_init,angle2_calc,angle3_init,angle3_calc,angle4_init,angle4_calc,angle5_init,angle5_calc,count)
fprintf("Loss Angle1:%f,Angle2:%f,Angle3:%f,Angle4:%f,Angle5:%f",loss1,loss2,loss3,loss4,loss5);
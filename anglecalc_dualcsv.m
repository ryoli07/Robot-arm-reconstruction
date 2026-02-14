%% アーム情報・初期値 -------------------------------------------------------
range = 8;
gOff = 8192/range;  %1gの値
fs = 625;  %サンプリングレート
dt = 1/fs;
gw = 9.80665; %重力
l1 = [0.195;0;0];   %第1アーム長
l2 = [0.205;0;0];   %第2アーム長
axis1 = 3;   %1軸目回転軸（x=1,y=2,z=3)
axis2 = 3;   %2軸目回転軸（x=1,y=2,z=3)

[th1Next,th1Curr,th1Prev] = deal(zeros(3,1),zeros(3,1),zeros(3,1)); %計算角度(Next:求める,Curr:今,Prev:一つ前)
[th2Next,th2Curr,th2Prev] = deal(zeros(3,1),zeros(3,1),zeros(3,1));

%% データ読み込み
[file1,location1] = uigetfile('.csv');
[file2,location2] = uigetfile('.csv');
meas1Raw = readmatrix(strcat(location1,file1));
meas2Raw = readmatrix(strcat(location2,file2));
[measSize1,~] = size(meas1Raw);
[measSize2,~] = size(meas2Raw);
measSize = min(measSize1,measSize2);
thList = zeros(measSize,2);
meas1 = [-meas1Raw(1:measSize,3),meas1Raw(1:measSize,4),meas1Raw(1:measSize,5)];
meas2 = [-meas2Raw(1:measSize,3),meas2Raw(1:measSize,4),meas2Raw(1:measSize,5)];
meas1(:,2) = meas1(:,2) - mean(meas1(:,2));
meas2(:,2) = meas2(:,2) - mean(meas2(:,2));
meas1 = meas1 / gOff * gw;
meas2 = meas2 / gOff * gw;

gradY = 0.028;
% gradX = 0.011;
gradX = 0.005;
Ry = [cos(gradY),0,-sin(gradY);0,1,0;sin(gradY),0,cos(gradY)];
Rx = [1,0,0;0,cos(gradX),sin(gradX);0,-sin(gradX),cos(gradX)];
gwY = Ry * [0;0;gw];
gwXY = Rx * gwY;

%%  ループ ----------------------------------------------------------------
for k = 1:measSize
    meas1K = [meas1(k,1);meas1(k,2);0];
    meas2K = [meas2(k,1);meas2(k,2);0];
    R2 = Rotate(th2Curr,axis2);
    
    Rz = [cos(thCurr(axis1)),sin(thCurr(axis1)),0;-sin(thCurr(axis1)),cos(thCurr(axis1)),0;0,0,1];
    gradZ = Rz * gwXY;
    measK1(1) = meas1K(1) - gradZ(1);
    measK1(2) = meas1K(2) - gradZ(2);
    meas2K(1) = meas2K(1) - gradZ(1);
    meas2K(2) = meas2K(2) - gradZ(2);

    [th1Next,th2Next] = Newton(th1Next,th1Curr,th1Prev,th2Next,th2Curr,th2Prev,meas1K,meas2K,l1,l2,axis1,axis2,R2,dt);
    thList(k,1) = th1Next(axis1);
    thList(k,2) = th2Next(axis2);
   
    th1Prev = th1Curr;
    th1Curr = th1Next;
    th2Prev = th2Curr;
    th2Curr = th2Next;  
end

plot(thList);
legend('th1', 'th2')

%% 関数定義 ----------------------------------------------------------------
function [th1Next,th2Next] = Newton(th1Next,th1Curr,th1Prev,th2Next,th2Curr,th2Prev,meas1,meas2,l1,l2,axis1,axis2,R2,dt)
    count = 0;
    while count < 100
        count = count + 1;
        [o1,a1] = DiffEq(th1Next,th1Curr,th1Prev,dt);
        [o2,a2] = DiffEq(th2Next,th2Curr,th2Prev,dt);
        [lt,J] = Diff(o1,o2,a1,a2,R2,dt,l1,l2,meas1,meas2,axis1,axis2);
        J = J + eye(2)*0.1;
        if abs(lt) < 1E-3
            break
        else
            newton  = [th1Next(axis1);th2Next(axis2)] - J \ lt;
            th1Next(axis1) = newton(1);
            th2Next(axis2) = newton(2);
        end    
    end
    fprintf('th1:%f,th2:%f,count:%d\n',th1Next(axis1),th2Next(axis2),count)
end


% 差分方程式 
function [om,al] = DiffEq(thNext,thCurr,thPrev,dt)
    om = (thNext - thCurr) / dt;
    al = (thNext - 2*thCurr + thPrev) / dt^2;
end

% スキューシンメトリック変換
function sk = Skew(vec) 
    sk = [0 -vec(3) vec(2); vec(3) 0 -vec(1); -vec(2) vec(1) 0];
end

% 回転行列
function R = Rotate(th,axis) 
    c = cos(th(axis));
    s = sin(th(axis));
    if axis == 1
        R = [1 0 0; 0 c -s; 0 s c];
    elseif axis == 2
        R = [c 0 s; 0 1 0; -s 0 c];
    elseif axis ==3
        R = [c -s 0; s c 0; 0 0 1];
    end
end

% 先端加速度計算
function [acc1,acc2] = Acc(l1,l2,R2,om1,om2,al1,al2)

    acc1 = Skew(om1)*Skew(om1)*l1 + Skew(al1)*l1;
    acc2 = R2'*(Skew(om1)*Skew(om1)*(l1+R2*l2) + Skew(al1)*(l1+R2*l2) + 2*Skew(om1)*R2*Skew(om2)*l2) + Skew(om2)*Skew(om2)*l2 + Skew(al2)*l2;
end

% 微分計算関数
function [lt,J] = Diff(om1,om2,al1,al2,R2,dt,l1,l2,meas1,meas2,axis1,axis2)  
    
    [acc1,acc2] = Acc(l1,l2,R2,om1,om2,al1,al2);
    % 誤差（ベクトル）
    err1 = acc1 - meas1;
    err2 = acc2 - meas2;

    % 回転軸設定
    e1 = zeros(3,1);
    e1(axis1) = 1;
    e2 = zeros(3,1);
    e2(axis2) = 1;
    
    % 角速度・角加速度 微分
    o1_d = e1/dt;
    o2_d  = e2/dt;
    a1_d = e1/dt/dt;
    a2_d = e2/dt/dt;
    
    % 一階微分（Diff数値はθ1,θ2)
    err1Diff1 = (Skew(om1)*(Skew(l1))'+(Skew(Skew(om1)*l1))')*o1_d + (Skew(l1))'*a1_d;
    err2Diff1 = R2'*(Skew(om1)*(Skew(l1+R2*l2))'+(Skew(Skew(om1)*(l1+R2*l2)))' + 2*(Skew(R2*Skew(om2)*l2))')*o1_d + R2'*(Skew(l1+R2*l2))'*a1_d;
    err2Diff2 = (2*R2'*Skew(om1)*R2*(Skew(l2))' + Skew(om2)*(Skew(l2))' + (Skew(Skew(om2)*l2))')*o2_d + (Skew(l2))'*a2_d ;
    err1Diff = [err1Diff1,zeros(3,1)];
    err2Diff = [err2Diff1,err2Diff2];

    % 二階微分
    err1Diff11 = o1_d' * (Skew(l1)*Skew(err1) + (Skew(err1))'*(Skew(l1))') * o1_d;
    err2Diff11 = o1_d' * (Skew(l1+R2*l2)*Skew(R2*err2) + (Skew(R2*err2))'*(Skew(l1+R2*l2))') * o1_d ;
    err2Diff12 = o1_d' * (2*(Skew(R2*err2))'*R2*(Skew(l2))') * o2_d;
    err2Diff21 = o2_d' * (2*Skew(l2)*R2'*Skew(R2*err2)) * o1_d;
    err2Diff22 = o2_d' * (Skew(l2)*Skew(err2) + (Skew(err2))'*(Skew(l2))') * o2_d;
    
    J1 = err1Diff'*err1Diff + err2Diff'*err2Diff;
    J2 = [err1Diff11 + err2Diff11,err2Diff12;err2Diff21,err2Diff22];
    J = J1 + J2;
    
    % 誤差二乗和の一階微分
    lt1 = err1Diff1'*err1 + err2Diff1'*err2;
    lt2 = err2Diff2'*err2; 
    lt = [lt1;lt2];
end

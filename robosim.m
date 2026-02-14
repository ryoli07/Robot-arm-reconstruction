%% アーム情報・初期値 -------------------------------------------------------
l1 = 0.21; l2 = 0.21;
l1Vec = [l1;0;0]; l2Vec = [l2;0;0];
axis1 = 3;   %1軸目回転軸（x=1,y=2,z=3)
axis2 = 3;   %2軸目回転軸（x=1,y=2,z=3)
fs  = 125;   %サンプリングレート
dt = 1/fs; 
N = 6250; %ステップ数
figTime = (0:N-1) * dt;

movement = 1; %運動選択（1軸往復=1,2軸往復=2,1軸回転=3,2軸回転=4)

[th1Next,th1Curr,th1Prev] = deal(zeros(3,1)); %計算角度(Next:求める,Curr:今,Prev:一つ前)
[th2Next,th2Curr,th2Prev] = deal(zeros(3,1));

tol = 0.00001; %ニュートン法の許容誤差

% シミュレーション運動設定
motion.th1 = pi/6; %往復の角度
motion.th2 = pi/5;
motion.om1 = pi/4; %各速度
motion.om2 = pi/6; 

ltList = zeros(1,100000);
figureCount = 0;
thList = zeros(N,2);
thMeasList = zeros(N,2);

%%  ループ ----------------------------------------------------------------
tic
for k = 1:N
    % テスト角度更新
    t = (k-1)*dt;

    [meas1,meas2,th1Meas,th2Meas] = AccMeas(motion,l1Vec,l2Vec,t,axis1,axis2,movement);

    % 角度推定計算
    th1Prev = th1Curr;
    th1Curr = th1Next;
    th2Prev = th2Curr;
    th2Curr = th2Next;
    R2 = Rotate(th2Curr,axis2);

     if k ==1
        th1Prev(axis1) = motion.th1 * sin(-motion.om1*dt);
        if movement == 2
            th2Prev(axis2) = motion.th2 * sin(-motion.om2*dt);
        end
     end

    [th1Next,th2Next] = Newton(th1Next,th1Curr,th1Prev,th2Next,th2Curr,th2Prev,meas1,meas2,R2,axis1,axis2,l1Vec,l2Vec,dt,tol);

    thList(k,1) = th1Next(axis1);
    thList(k,2) = th2Next(axis2);
    thMeasList(k,1) = th1Meas;
    thMeasList(k,2) = th2Meas;
end
toc

%% figure -----------------------------------------------------------------

hold off;

%角度推移プロット
plot(figTime,thMeasList(:,1),"Color","#f2cf01","LineWidth",2.0); hold on;
plot(figTime,thMeasList(:,2),"Color","#56a764","LineWidth",2.0);
plot(figTime,thList(:,1),"Color","#0074bf","LineWidth",2.0);
plot(figTime,thList(:,2),"Color","#c93a40","LineWidth",2.0);
legend({"test angle1","test angle2","calc angle1","calc angle2"},"FontSize",16);
grid on;
ax = gca; 
ax.LineWidth = 1.5;
ax.FontSize = 12;
xlabel("time(s)","FontSize",20);
ylabel("angle(rad)","FontSize",20);
ax.FontWeight = "bold";
ylim([-1.5,1.5])
exportgraphics(ax,'imgSimAngle2-3.pdf','ContentType','vector')


% 収束回数プロット
% semilogy(ltList(1:50),'-',"Color","#0074bf","LineWidth",2.0); hold on;
% semilogy(ltList(1:50),'o', 'MarkerEdgeColor','black','MarkerSize', 10,"LineWidth",1.5); hold on;
% yline(tol,'r--','y=0.00001','LineWidth',2.0,'FontSize', 14,'FontWeight', 'bold');
% grid on;
% ax = gca; 
% ax.LineWidth = 1.5;
% ax.FontSize = 12;
% ax.GridAlpha = 0.3;
% ax.XMinorGrid = 'off';  
% ax.YMinorGrid = 'off';
% xlabel("newton method step","FontSize",20);
% ylabel("error","FontSize",20);
% ax.FontWeight = "bold";
% exportgraphics(ax,'simErr1.pdf','ContentType','vector')

% 軌跡プロット
% h1 = plot([0,l1,l1+l2], [0,0,0],'-',"Color","#c93a40","LineWidth",2.0); hold on;
% h2 = plot([0,l1,l1+l2],[0,0,0],'o',"Color","#0074bf",markersize=12); 
% xlim([-(l1+l2),l1+l2]);
% ylim([-(l1+l2),l1+l2]);
% for i = 1:N
%     meas1x =  l1*cos(thMeasList(i,1));
%     meas1y =  l1*sin(thMeasList(i,1));
%     meas2x = l1*cos(thMeasList(i,1)) + l2*cos(thMeasList(i,1)+thMeasList(i,2));
%     meas2y = l1*sin(thMeasList(i,1)) + l2*sin(thMeasList(i,1)+thMeasList(i,2));
%     calc1x =  l1*cos(thList(i,1));
%     calc1y =  l1*sin(thList(i,1));
%     calc2x = l1*cos(thList(i,1)) + l2*cos(thList(i,1)+thList(i,2));
%     calc2y = l1*sin(thList(i,1)) + l2*sin(thList(i,1)+thList(i,2));
%     set(h1, 'XData', [0,meas1x,meas2x], 'YData', [0,meas1y,meas2y])
%     set(h2, 'XData', [0,calc1x,calc2x], 'YData', [0,calc1y,calc2y])
%     pause(dt)
% end

% thErr = mean(abs(thTestList-thList));
% disp(thErr)

%% 関数定義 ----------------------------------------------------------------

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

function [th1Next,th2Next] = Newton(th1Next,th1Curr,th1Prev,th2Next,th2Curr,th2Prev,meas1,meas2,R2,axis1,axis2,l1Vec,l2Vec,dt,tol)
    count = 0;
    while count < 100 %ニュートン法計算
        count = count +1;
        [o1,a1] = DiffEq(th1Next,th1Curr,th1Prev,dt);
        [o2,a2] = DiffEq(th2Next,th2Curr,th2Prev,dt);
        [lt,J] = Diff(o1,o2,a1,a2,R2,dt,l1Vec,l2Vec,meas1,meas2,axis1,axis2);
        J = J + eye(2)*0.1;
        diff =  J \ lt;
         newton  = [th1Next(axis1);th2Next(axis2)] - diff;
        if abs(norm(lt))< tol
            break
        end
        th1Next(axis1) = newton(1);
        th2Next(axis2) = newton(2);
    end
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

% シミュレーション加速度
function [acc1,acc2,th1,th2] = AccMeas(m,l1Vec,l2Vec,t,axis1,axis2,movement)

    % 変数初期化
    [th2Vec,om1Vec,om2Vec,al1Vec,al2Vec] = deal(zeros(3,1)); 
    [th2,om2,al1,al2] = deal(0);

    % axis1 角度・角速度・各加速度
    if movement <= 2
        th1 = m.th1 * sin(m.om1*t);
        om1 = m.th1 * m.om1  *cos(m.om1*t);
        al1 = -m.th1 * m.om1^2 * sin(m.om1*t);
    else
        th1 = m.om1*t;
        om1 = m.om1;
    end

    % axis2 角度・角速度・各加速度
    if movement == 2
        th2 = m.th2 * sin(m.om2*t);
        om2 = m.th2 * m.om2  *cos(m.om2*t);
        al2 = -m.th2 * m.om2^2 * sin(m.om2*t);
    elseif movement == 4
        th2 = m.om2*t;
        om2 = m.om2;
    end
    th2Vec(axis2) = th2;
    om1Vec(axis1) = om1; om2Vec(axis2) = om2;
    al1Vec(axis1) = al1; al2Vec(axis2) = al2;
    R2 = Rotate(th2Vec,axis2);
    
    % 加速度
    [acc1,acc2] = Acc(l1Vec,l2Vec,R2,om1Vec,om2Vec,al1Vec,al2Vec);
end

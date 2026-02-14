%% 基本データ --------------------------------------------------------------
range = 8;
gOff = 8192/range;  %1gの値
fs = 125;  %サンプリングレート
dt = 1/fs;
gw = 9.80665; %重力
L = 0.42;  %第1アーム長
[thNext,thCurr,thPrev] = deal(0,0,0); %計算角度(Next:求める,Curr:今,Prev:一つ前)
% ベクトル用↓
% L = [0.42;0;0];   %第1アーム長
% axis1 = 3;   %1軸目回転軸（x=1,y=2,z=3)
% [thNext,thCurr,thPrev] = deal(zeros(3,1),zeros(3,1),zeros(3,1)); %計算角度(Next:求める,Curr:今,Prev:一つ前)

%% データ読み込み ----------------------------------------------------------
[file,location] = uigetfile('.csv');
measRaw = readmatrix(strcat(location,file));
[measSize,~] = size(measRaw);
thList = zeros(measSize,1);
meas = [-measRaw(:,3),-measRaw(:,4),measRaw(:,5)];
meas(:,1) = meas(:,1) + 174.46;
meas(:,2) = meas(:,2) + 110;
figTime = (0:measSize-1) * dt;
measFig = meas / gOff * gw;
meas = meas / gOff * gw / L;

% ベクトル用↓
% meas = meas / gOff * gw;

%%  ループ ----------------------------------------------------------------
for k = 1:measSize
    mx = meas(k,1);
    my = meas(k,2);
    mz = meas(k,3);
    thNext = Newton(thNext,thCurr,thPrev,mx,my,mz,dt);
    thList(k,1) = thNext;
    thPrev = thCurr;
    thCurr = thNext;
end

%% ベクトル形式ループ -------------------------------------------------------
% for k = 1:measSize
%     measK = [meas(k,1);meas(k,2);0];
%     thNext = NewtonVec(thNext,thCurr,thPrev,measK,L,axis1,dt);
%     thList(k,1) = thNext(axis1);
%     thPrev = thCurr;
%     thCurr = thNext;
% end

%% フィルタ ----------------------------------------------------------------

Rp = 0.00057565;
Rst = 1e-3;

% ハイパスフィルタ
hpTap = 300;
hpNum = firceqrip(hpTap, 0.01/(fs/2),[Rp,Rst], 'high');
hpf = dsp.FIRFilter('Numerator', hpNum);
thListHpf = hpf(thList);

% 改良版ハイパスフィルタ
hpnTap = 400;
hpnNum = firceqrip(hpnTap, 0.01/(fs/2),[Rp,Rst]); 
hpnNum = hpnNum/sum(hpnNum); 
hpfNew = dsp.FIRFilter("Numerator",hpnNum);
thListHpfNew = hpfNew(thList);
thListHpfNew = thList - [thListHpfNew(floor((hpnTap)/2)+1:end);zeros(floor((hpnTap)/2),1)];

hpnNum2 = -hpnNum;
hpnNum2(1) = hpnNum2(1) + 1;
hpfNew2 = dsp.FIRFilter("Numerator",hpnNum2);

%DCBlocker
dcblk = dsp.DCBlocker('NormalizedBandwidth', 0.005,'Order', 3);
thListDcblk = dcblk(thList);

%% フィルタ特性
% h = fvtool(dcblk);
% xlim(h.CurrentAxes, [0 0.14]);

%% 角度データプロット
% hold off;
% % plot(figTime(1:measSize-300),thList(1:measSize-300),"Color","#0074bf","LineWidth",2.0); hold on;
% % plot(figTime(1:measSize-300),thListHpf(1:measSize-300),"Color","#f2cf01","LineWidth",2.0);
% plot(figTime(1:measSize-300),thListHpfNew(1:measSize-300),"Color","#c93a40","LineWidth",2.0);
% % plot(figTime(1:measSize-300),thListDcblk(1:measSize-300),"Color","#c93a40","LineWidth",2.0);
% 
% % legend({"RAW","HPF","New-HPF"},"FontSize",16)
% % legend({"RAW","HPF"},"FontSize",16)
% grid on;
% ax = gca;
% ax.LineWidth = 1.5;
% ax.FontSize = 12;
% ylabel("angle(rad)","FontSize",20);
% xlabel("time(s)","FontSize",20);
% ax.FontWeight = "bold";
% exportgraphics(ax,'angleCrash.pdf','ContentType','vector')

rawEval = polyfit(figTime(1:measSize-300),thList(1:measSize-300), 1);  % 1次近似（直線フィッティング）
hpfEval = polyfit(figTime(1:measSize-300),thListHpf(1:measSize-300), 1);
hpfNewEval = polyfit(figTime(1:measSize-300),thListHpfNew(1:measSize-300), 1);
hpfResult = (1 - abs(hpfEval(1)) / abs(rawEval(1))) * 100;
hpfNewResult = (1 - abs(hpfNewEval(1)) / abs(rawEval(1))) * 100;

%% 加速度データプロット
plot(figTime,measFig);
legend({"x","y","z"},"FontSize",16)
xlim([10 40])
grid on;
ax = gca;
ax.LineWidth = 1.5;
ax.FontSize = 12;
xlabel("time(s)","FontSize",20);
ylabel("acc(m/s^2)","FontSize",20);
ax.FontWeight = "bold";
exportgraphics(ax,'accCrash.pdf','ContentType','vector')


%% writematrix -----------------------------------------------------------
% inputName = input("File name :","s");
% timeStamp = datetime("now");
% timeStamp.Format = "MMddHHmmss";
% timeStamp = string(timeStamp);
% exportFileName = strcat(inputName,"_",timeStamp,".csv");
% writematrix(thListFilA,exportFileName);
 
% LPFタップ数検証 -----------------------------------------------------
% tapList = [300,400,500,700,1000];
% ax = gca;
% ax.LineWidth = 1.5;
% ax.FontSize = 12;
% ax.FontWeight = "bold";
% ax.ColorOrder = [
%     242 207 1 %#f2cf01
%     0 116 191 %#0074bf
%     86 167 100 %#56a764
%     201 58 64 %#c93a40
%     148 96 160 %#9460a0
%     ]/255;
% ax.NextPlot = 'replacechildren';
% for i = 1:length(tapList)
%     hpnTap = tapList(i);
%     hpnNum = firceqrip(hpnTap, 0.01/(fs/2),[Rp,Rst]); 
%     hpnNum = hpnNum/sum(hpnNum); 
%     hpfNew = dsp.FIRFilter("Numerator",hpnNum);
%     thListHpfNew = hpfNew(thList);
%     thListHpfNew = thList - [thListHpfNew(floor((hpnTap)/2)+1:end);zeros(floor((hpnTap)/2),1)];
%     plot(figTime(1:3000),thListHpfNew(1:3000),"LineWidth",2.0); hold on;
%     ylabel("angle(rad)","FontSize",20)
%     xlabel("time(s)","FontSize",20)
% end
% legend("300","400","500","700","1000")
% grid on;
% exportgraphics(ax,'filtap.pdf','ContentType','vector')

%% 関数定義 ----------------------------------------------------------------

% ニュートン法
function thNext = Newton(thNext,thCurr,thPrev,mx,my,mz,dt)  
    count = 0;
    while count < 100
        count = count + 1;
        [om,al] = DiffEq(thNext,thCurr,thPrev,dt); 
        lt = (-om*om - mx) * (-2*om/dt) + (al-my)/dt/dt;
        if abs(lt)< 1E-3
            break;
        else
            J = (-2*om/dt) * (-2*om/dt) + (-om*om - mx) * (-2/dt/dt) + 1/dt/dt/dt/dt; 
            thNext = thNext - lt / J;
        end 
    end
end

% 差分方程式 
function [om,al] = DiffEq(thNext,thCurr,thPrev,dt)
    om = (thNext - thCurr) / dt;
    al = (thNext - 2*thCurr + thPrev) / dt/dt;
end

% ニュートン法 ベクトル形式
function thNext = NewtonVec(thNext,thCurr,thPrev,meas,L,axis1,dt)
    count = 0;
    while count < 100
        count = count + 1;
        [om,al] = DiffEq(thNext,thCurr,thPrev,dt); 
        acc1 = Skew(om)*Skew(om)*L + Skew(al)*L;
        err1 = acc1 - meas;  % 誤差（ベクトル）
        e1 = zeros(3,1);  % 回転軸設定
        e1(axis1) = 1;
        o1_d = e1/dt;   % 角速度・角加速度 微分
        a1_d = e1/dt/dt;
        err1Diff1 = (Skew(om)*(Skew(L))'+(Skew(Skew(om)*L))')*o1_d + (Skew(L))'*a1_d;  % 一階微分（Diff数値はθ1,θ2)
        err1Diff11 = o1_d' * (Skew(L)*Skew(err1) + (Skew(err1))'*(Skew(L))') * o1_d;
        lt = (err1')*err1Diff1;
        if abs(lt)< 1E-3
            break
        else
            J = (err1Diff1')*err1Diff1 + err1Diff11;
            newton  = thNext(axis1) - lt / J;
            thNext(axis1) = newton;  
        end   
    end
end

% スキューシンメトリック変換
function sk = Skew(vec) 
    sk = [0 -vec(3) vec(2); vec(3) 0 -vec(1); -vec(2) vec(1) 0];
end

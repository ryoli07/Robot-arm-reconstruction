%% 初期値・基本データ -------------------------------------------------------

global l1 l2 axis1 axis2 dt tc th1Next th2Next th1Curr th2Curr th1Prev th2Prev
global ltNorm err1List figurecount acc1List meas1List sendCount dataList thList

% アーム情報
l1 = [0.0378;0;0];   %第1アーム長
l2 = [0.0467;0;0];   %第２アーム長
axis1 = 3;   %1軸目回転軸（x=1,y=2,z=3)
axis2 = 3;   %2軸目回転軸（x=1,y=2,z=3)
[th1Next,th1Curr,th1Prev] = deal(zeros(3,1),zeros(3,1),zeros(3,1)); %計算角度
[th2Next,th2Curr,th2Prev] = deal(zeros(3,1),zeros(3,1),zeros(3,1));
th1Next(axis1) = 0;
th2Next(axis2) = 0;

% データ取得情報
d = daq("ni");
fs = 100;
d.Rate = fs;
scanCount = 100;
device = "Dev2";
dt = 1 / d.Rate;
dataOff = 0.9;
ch1 = addinput(d,"Dev2","ai0","Voltage");
ch2 = addinput(d,"Dev2","ai1","Voltage");
ch3 = addinput(d,"Dev2","ai2","Voltage");
ch4 = addinput(d,"Dev2","ai3","Voltage");
ch5 = addinput(d,"Dev2","ai4","Voltage");
ch6 = addinput(d,"Dev2","ai5","Voltage");
ch1.TerminalConfig = "SingleEnded";
ch2.TerminalConfig = "SingleEnded";
ch3.TerminalConfig = "SingleEnded";
ch4.TerminalConfig = "SingleEnded";
ch5.TerminalConfig = "SingleEnded";
ch6.TerminalConfig = "SingleEnded";

%フィルタ情報
N = 50;
fp = 2;
rp = 0.00057565;
rst = 1E-5;
eqnum = firceqrip(N, fp/fs*2, [rp rst], 'passedge');
dcblker = dsp.DCBlocker("NormalizedBandwidth", 0.00001, "Order", 10);
lpf = dsp.FIRFilter('Numerator',eqnum); 

% 記録用情報
figurecount = 0;
sendCount = 0;
ltNorm = zeros(100000,1);
err1List = zeros(100000,3);
acc1List = zeros(100000,3);
meas1List = zeros(100000,3);
dataList = zeros(100000,6);
thList = zeros(100000,2);

% UI・グラフ情報
fig = uifigure("Name", "DAQ", "Position", [100 100 640 200]);
txt = uilabel(fig,...
    'Position',[10 450 200 20], 'Text', "");
avg = uilabel(fig,...
    'Position',[10 420 200 20], 'Text', "");
% ax = uiaxes("Parent", fig, "Units", "pixels", "Position", [80, 80, 500, 350]);
sscnt = uibutton(fig, "Text", "STOP", "Position", [10 10 100 20], "ButtonPushedFcn", @(btn, event)stopCapture(btn, d));
qbtn = uibutton(fig, "Text", "QUIT", "Position", [500 10 100 20], "ButtonPushedFcn", @(btn, event)quit(btn, d, fig));

%ソケット情報
% tc = tcpclient("127.0.0.1",50009);

%% データ取得・送信 --------------------------------------------------------

d.ScansAvailableFcn = @(src,evt)sendData(src,dataOff,dcblker,lpf);
d.ScansAvailableFcnCount = scanCount;
start(d,"continuous");

% データ送信関数
function sendData(src,dataOff,dcblker,lpf)
    global l1 l2 axis1 axis2 dt tc th1Next th2Next th1Curr th2Curr th1Prev th2Prev
    global ltNorm err1List figurecount acc1List meas1List sendCount dataList thList

    dataRaw = read(src,src.ScansAvailableFcnCount,"OutputFormat","Matrix");
    
    % ch1Off = 0.900401;
    % ch2Off = 0.901939;
    % ch3Off = 0.900412;
    % ch4Off = 0.895166;
    % ch5Off = 0.894457;
    % ch6Off = 0.908454;
    % dataRaw(:,1) = dataRaw(:,1) - ch1Off;
    % dataRaw(:,2) = dataRaw(:,2) - ch2Off;
    % dataRaw(:,3) = dataRaw(:,3) - ch3Off;
    % dataRaw(:,4) = dataRaw(:,4) - ch4Off;
    % dataRaw(:,5) = dataRaw(:,5) - ch5Off;
    % dataRaw(:,6) = dataRaw(:,6) - ch6Off;
  
    dataRaw = dataRaw - dataOff;
    dataMean = dataRaw - mean(dataRaw);

    dataFilt = lpf(dataMean);
    dataDC = dataFilt - dcblker(dataFilt);
    data = dataDC/80e-3 * 9.80665;
    data(:,3) = 0;
    data(:,6) = 0;

    for i = 1:size(data,1)
        sendCount = sendCount +1;
        disp(data(i,:));
        meas1 = [data(i,1),data(i,2),0];
        meas2 = [data(i,4),data(i,5),0];

        dataList(sendCount,:) = data(i,:);
        
        th1Prev = th1Curr;
        th1Curr = th1Next;
        th2Prev = th2Curr;
        th2Curr = th2Next;
        R2 = Rotate(th2Curr,axis2);
        
        count = 0;
        while count < 100  %ニュートン法計算
            figurecount = figurecount +1;
            count = count +1;
    
            [o1,a1] = DiffEq(th1Next,th1Curr,th1Prev,dt);
            [o2,a2] = DiffEq(th2Next,th2Curr,th2Prev,dt);
    
            [Lt,J,err1,err2,acc1,acc2] = Diff(o1,o2,a1,a2,R2,dt,l1,l2,meas1,meas2,axis1,axis2);
            J = J + eye(2)*0.1;
            
            diff =  J \ Lt;
             newton  = [th1Next(axis1);th2Next(axis2)] - diff;  
            
            if abs(norm(Lt))< 0.00000001
                % disp(count)
                thList(sendCount,1) = th1Next(axis1);
                thList(sendCount,2) = th2Next(axis2);
                break
            end
            err1List(figurecount,:) = err1;
            meas1List(figurecount,:) = meas1;
            acc1List(figurecount,:) = acc1;
            ltNorm(figurecount,:) = norm(Lt);
            th1Next(axis1) = newton(1);
            th2Next(axis2) = newton(2);
        end
    
        % [th1New,th2New] = Calc(meas1,meas2,th1New,th2New,th1Curr,th2Curr,th1Prev,th2Prev,axis1,axis2,dt);
        
        % th1_100x = int32(floor(th1Next(axis1)*100)); %ビットスライド
        % th1Send = th1_100x*65536;
        % th2Send = int32(floor(th2Next(axis2)*100));
        % send_data = th1Send + th2Send;
        fprintf('th1Next:%f th2Next:%f\n',th1Next(axis1),th2Next(axis2))
        % write(tc, send_data, 'int32'); %TCP送信
    end
end

%% 関数定義 ---------------------------------------------------------------

% 差分方程式
function [om,al] = DiffEq(thNext, thCurr, thPrev, dt)
    om = (thNext - thCurr) / dt;
    al = (thNext - 2*thCurr + thPrev) / dt^2;
end

% スキューシンメトリック変換-
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

%先端加速度計算
function [acc1,acc2] = Acc(l1,l2,R2,om1,om2,al1,al2)  

    acc1 = Skew(om1)*Skew(om1)*l1 + Skew(al1)*l1;
    acc2 = R2'*(Skew(om1)*Skew(om1)*(l1+R2*l2) + Skew(al1)*(l1+R2*l2) + 2*Skew(om1)*R2*Skew(om2)*l2) + Skew(om2)*Skew(om2)*l2 + Skew(al2)*l2;
end

% 微分計算関数
function [Lt,J,err1,err2,acc1,acc2] = Diff(om1,om2,al1,al2,R2,dt,l1,l2,meas1,meas2,axis1,axis2)  
    
    [acc1,acc2] = Acc(l1,l2,R2,om1,om2,al1,al2);
    
    % 誤差（ベクトル）
    err1 = acc1 - meas1'; 
    err2 = acc2 - meas2';

    % 回転軸設定
    e1 = zeros(3,1);  
    e1(axis1) = 1;
    e2 = zeros(3,1);
    e2(axis2) = 1;
    
    % 角速度・角加速度 微分
    o1_d = e1/dt;
    o2_d = e2/dt;
    a1_d = e1/dt/dt;
    a2_d = e2/dt/dt;
   
    % 一階微分
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
    
    % ヤコビアン
    J1 = err1Diff'*err1Diff + err2Diff'*err2Diff;
    J2 = [err1Diff11 + err2Diff11,err2Diff12;err2Diff21,err2Diff22];
    J = J1 + J2;
    
    % 誤差二乗和の一階微分
    Lt1 = err1Diff1'*err1 + err2Diff1'*err2;
    Lt2 = err2Diff2'*err2; 
    Lt = [Lt1;Lt2];
end

%quitボタン関数
function quit(btn,s,fig)
    stop(s);
    close(fig);
end

%stopボタン関数
function stopCapture(btn, s)
    global adata;
    global txt; 
    if strcmp(btn.Text, 'STOP') 
        stop(s);
        adata = [];
        btn.Text = 'ReSTART';
    else
        start(s, "continuous");
        btn.Text = 'STOP';
    end
end
clear;
%% 基本データ

global readCount dataFiltList dataRawList dataGList dataDCList

% データ取得情報
% global cont;
% cont = logical(true);
dev = daqlist();
d = daq("ni");
fs = 100;
d.Rate = fs;
scanCount = 100;
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

% フィルタ情報
N = 50;
fp = 2;
rp = 0.00057565;
rst = 1E-5;
eqnum = firceqrip(N, fp/fs*2, [rp rst], 'passedge');
dcblker = dsp.DCBlocker("NormalizedBandwidth", 0.00001, "Order", 10);
lpf = dsp.FIRFilter('Numerator',eqnum); 

% 記録用情報
dataFiltList = zeros(100000,6);
dataRawList = zeros(100000,6);
dataDCList = zeros(100000,6);
dataGList = zeros(100000,6);
readCount = 0;

% UI・グラフ情報
fig = uifigure("Name", "DAQ", "Position", [100 100 640 480]);
ax = uiaxes("Parent", fig, "Units", "pixels", "Position", [80, 80, 500, 350]);
sscnt = uibutton(fig, "Text", "STOP", "Position", [10 10 100 20], "ButtonPushedFcn", @(btn, event)stopCapture(btn, d, fdtstr));
qbtn = uibutton(fig, "Text", "QUIT", "Position", [500 10 100 20], "ButtonPushedFcn", @(btn, event)quit(btn, d, fig));
nmuch = 2;

%% データ取得

d.ScansAvailableFcn = @(src, evt)plotData(src,ax,dataOff,dcblker,lpf);
d.ScansAvailableFcnCount = scanCount;
start(d, "continuous");

function plotData(src,ax,dataOff,dcblker,lpf)
    global readCount dataFiltList dataRawList dataGList dataDCList;
    dataRaw = read(src, src.ScansAvailableFcnCount, "OutputFormat", "Matrix");
    [r,c] = size(dataRaw);

    dataRaw = dataRaw - dataOff;
    dataMean = dataRaw - mean(dataRaw);
    dataFilt = lpf(dataMean);
    % dataDC = dataFilt - dcblker(dataFilt);
    dataDC = dcblker(dataFilt);
    dataG = dataDC/80e-3 * 9.80665;
    
    for i = 1:size(dataRaw,1)
        readCount = readCount +1;
        dataFiltList(readCount,:) = dataFilt(i,:);
        dataRawList(readCount,:) = dataRaw(i,:);
        dataDCList(readCount,:) = dataDC(i,:);
        dataGList(readCount,:) = dataG(i,:);
        disp(dataRaw(i,:));
    end 

    Fs = src.Rate;
    t = (0:1/Fs:(r-1)/Fs);
    f = (0:r/src.Rate:src.Rate-1/src.Rate);
    hold(ax,'off')
    plot(ax, t, dataRaw(:,1),'r'); hold(ax,'on');
    plot(ax, t, dataRaw(:,2),'b');
    plot(ax, t, dataRaw(:,3),'g');
    plot(ax, t, dataRaw(:,4),'y');
    plot(ax, t, dataRaw(:,5),'m');
    plot(ax, t, dataRaw(:,6),'w');
    ylim(ax, [-0.1 0.1]);
end

%% 関数定義

%stopボタン関数
function stopCapture(btn, d)
    % global adata;
    if strcmp(btn.Text, 'STOP')
        stop(d);
        % adata = [];
        btn.Text = 'ReSTART';
    else
        start(d, "continuous");
        btn.Text = 'STOP';
    end
end

% quitボタン関数
function quit(btn, d, fig)
    global dataRawList dataFiltList dataDCList readCount
    stop(d);
    writematrix(dataRawList(1:readCount,:),'dataRawList.csv');
    writematrix(dataFiltList(1:readCount,:),'dataFiltList.csv');
    writematrix(dataDCList(1:readCount,:),'dataDCList.csv');
    close(fig);
end
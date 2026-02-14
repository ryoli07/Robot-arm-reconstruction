clear;

% DAQデバイス一覧
dev = daqlist();

% DAQセクション作成
d = daq("ni");
d.Rate = 100; %サンプリングレート

% データリスト
global adata;
adata = [];

%チャンネル設定
ch1 = addinput(d,"Dev2","ai0","Voltage");
% ch2 = addinput(d,"Dev2","ai1","Voltage");
% ch3 = addinput(d,"Dev2","ai2","Voltage");
% ch4 = addinput(d,"Dev2","ai3","Voltage");
% ch5 = addinput(d,"Dev2","ai4","Voltage");
% ch6 = addinput(d,"Dev2","ai5","Voltage");

ch1.TerminalConfig = "SingleEnded";
% ch2.TerminalConfig = "SingleEnded";
% ch3.TerminalConfig = "SingleEnded";
% ch4.TerminalConfig = "SingleEnded";
% ch5.TerminalConfig = "SingleEnded";
% ch6.TerminalConfig = "SingleEnded";

% データ取得設定
stepCount = 600;
d.ScansAvailableFcn = @(src, evt)readData(src,stepCount);
d.ScansAvailableFcnCount = 10; %一度に読むスキャン数

% データ取得開始
start(d, "continuous");

global stepMemory
stepMemory = 0;

%% データ取得関数
function readData(src,stepCount)
    global adata stepMemory;
    stepMemory = stepMemory + 1;
    data = read(src, src.ScansAvailableFcnCount, "OutputFormat", "Matrix");
    adata = [adata; data];
    fprintf("%d : %f\n",stepMemory,data(end,:));

    if stepMemory >= stepCount
        stop(src);
        disp("DAQ取得終了");
        avgData = mean(adata,1);
        fprintf("平均値:%f",avgData);
    end
end

% ch1Off = 0.900401;
% ch2Off = 0.901939;
% ch3Off = 0.900412;
% ch4Off = 0.895166;
% ch5Off = 0.894457;
% ch6Off = 0.908454;

clear;
global cont;
global tc; 
tc = tcpclient("127.0.0.1",50008);
cont = logical(true);
dev = daqlist();
s = daq("ni");
s.Rate = 1000;
global adata;
adata = [];
fdt = datetime("now","TimeZone","Asia/Tokyo","Format","dd-MMM-uuuu HH:mm:ss.SSS");
fdtstr = string(fdt,"uuuuMMdd");
if exist(fdtstr, "dir") ~= 7
    mkdir(fdtstr);
end

fig = uifigure("Name", "DAQ", "Position", [100 100 640 480]);
ax = uiaxes("Parent", fig, "Units", "pixels", "Position", [80, 80, 500, 350]);
sscnt = uibutton(fig, "Text", "STOP", "Position", [10 10 100 20], "ButtonPushedFcn", @(btn, event)stopCapture(btn, s, fdtstr));
qbtn = uibutton(fig, "Text", "QUIT", "Position", [500 10 100 20], "ButtonPushedFcn", @(btn, event)quit(btn, s, fig));

nmuch = 2;
ch0 = addinput(s, 'Dev2', 'ai1', 'Voltage');
ch1 = addinput(s, 'Dev2', 'ai0', 'Voltage');
ch2 = addinput(s, 'Dev2', "ai2", 'Voltage');
ch0.TerminalConfig = 'SingleEnded';
ch1.TerminalConfig = 'SingleEnded';
ch2.TerminalConfig = 'SingleEnded';

% global count
% count = 100;
s.ScansAvailableFcn = @(src, evt)plotData(src,ax);
s.ScansAvailableFcnCount = 100;
%start(s,"Duration",20); 
start(s,"duration", 200); 
function plotData(src,ax)
    global adata;
    global tc; 
    data = read(src, src.ScansAvailableFcnCount, "OutputFormat", "Matrix");
    [r, c] = size(data);
    adata = [adata; data];
    Fs = src.Rate;
    t = (0:1/Fs:(r-1)/Fs);
    f = (0:r/src.Rate:src.Rate-1/src.Rate);
    data = (data - 0.9)*10; 
    data_sum = data(1:100,1:3);
    data_average = mean(data_sum); 
    [theta,phi] = Newton(data_average);

    theta_io = int32(floor(theta*100));
    theta_i = theta_io*65536;
    phi_i = int32(floor(phi*100));
    send_data = theta_i + phi_i;
    write(tc, send_data, 'int32'); 
    fprintf("data=%f theta=%d phi=%f\n",data_average,theta_io,phi_i)
end

function [theta,phi] = Newton(data)
    theta = 0.3;
    phi = 0.3;
    while true
        [Lt,Lp,Ltt,Ltp,Lpt,Lpp] = Diff(theta,phi,data);
        R_newton = [Ltt,Ltp;Lpt,Lpp];
        newton = [theta;phi] - R_newton\ [Lt;Lp];
        theta2 = newton(1,1);
        phi2 = newton(2,1);
        if abs(theta2 - theta) < 0.0001 && abs(phi2 - phi) < 0.0001
            break
        end
        theta = theta2;
        phi = phi2;
    end
end

function [Lt,Lp,Ltt,Ltp,Lpt,Lpp] = Diff(theta,phi,data) 
    e_vector = [0;-1;0];
    c_vector = data';

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

function stopCapture(btn, s, fdtstr)
    global adata;
    if strcmp(btn.Text, 'STOP')
        dt = datetime("now","TimeZone","Asia/Tokyo","Format","dd-MMM-uuuu HH:mm:ss.SSS");
        dtstr = string(dt,"uuuuMMdd_HHmmss");
        stop(s);
        adata = [];
        btn.Text = 'ReSTART';
    else
        start(s, "continuous");
        btn.Text = 'STOP';
    end
end

function quit(btn, s, fig)
    stop(s);
    close(fig);
end
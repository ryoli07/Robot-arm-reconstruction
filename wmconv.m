[file,location] = uigetfile('.csv',"","/home/mitsugi/Downloads/");
dataCSV = readmatrix(strcat(location,file));
[measSize,~] = size(dataCSV);
measRaw = dataCSV(:,18);
measRaw = measRaw*pi/180;
fs = 100; 
dt = 1/fs;
figTime = (0:measSize-1) * dt;
hold off;
plot(figTime,measRaw); hold on;
meas = measRaw - mean(measRaw);
plot(figTime,meas);
disp(file)
legend("raw","Filt")

%% writematrix -----------------------------------------------------------
inputName = input("File name :","s");
timeStamp = datetime("now");
timeStamp.Format = "MMddHHmmss";
timeStamp = string(timeStamp);
exportFileName = strcat(inputName,"_",timeStamp,".csv");
writematrix(meas,exportFileName);

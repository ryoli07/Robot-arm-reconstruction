% dir1 = uigetdir;
% dir2 = uigetdir;
dir1 = 'C:\Users\lio\Documents\autoid\Codes\Data\Experiment3\Data1';
dir2 = 'C:\Users\lio\Documents\autoid\Codes\Data\Experiment3\Data2';
num = 6;
hpfData1 = cell(1,num);
hpfData2 = cell(1,num);
gyroData1 = cell(1,num);
gyroData2 = cell(1,num);

for i = 1:num
    fileNameHpf1 = sprintf('hpf%d.csv',i-1);
    fileNameHpf2 = sprintf('hpf%d.csv',i-1);
    fileNameGyro1 = sprintf('gyro%d.csv',i-1);
    fileNameGyro2 = sprintf('gyro%d.csv',i-1);
    hpfData1{i} = readmatrix(strcat(dir1,"\",fileNameHpf1));
    hpfData2{i} = readmatrix(strcat(dir2,"\",fileNameHpf2));
    gyroData1{i} = readmatrix(strcat(dir1,"\",fileNameGyro1));
    gyroData2{i} = readmatrix(strcat(dir2,"\",fileNameGyro2));
    hpfData1{i} = hpfData1{i}(1:5000);
    hpfData2{i} = hpfData2{i}(1:5000);
    gyroData1{i} = resample(gyroData1{i},5,4);
    gyroData2{i} = resample(gyroData2{i},5,4);
    gyroData1{i} = gyroData1{i}(1:5000);
    gyroData2{i} = gyroData2{i}(1:5000);
end

refGyro1 = gyroData1{1};
refHpf1 = hpfData1{1};
refGyro2 = gyroData2{1};
refHpf2 = hpfData2{1};

for i = 2:num
    [cGyro1,lagGyro1] = xcorr(gyroData1{i},refGyro1);
    [~,idxGyro1] = max(cGyro1);
    shiftGyro1 = lagGyro1(idxGyro1);
    gyroData1{i} = circshift(gyroData1{i}, -shiftGyro1);

    [cGyro2,lagGyro2] = xcorr(gyroData2{i},refGyro2);
    [~,idxGyro2] = max(cGyro2);
    shiftGyro2 = lagGyro2(idxGyro2);
    gyroData2{i} = circshift(gyroData2{i}, -shiftGyro2);

    [cHpf1,lagHpf1] = xcorr(hpfData1{i},refHpf1);
    [~,idxHpf1] = max(cHpf1);
    shiftHpf1 = lagHpf1(idxHpf1);
    hpfData1{i} = circshift(hpfData1{i}, -shiftHpf1);

    [cHpf2,lagHpf2] = xcorr(hpfData2{i},refHpf2);
    [~,idxHpf2] = max(cHpf2);
    shiftHpf2 = lagHpf2(idxHpf2);
    hpfData2{i} = circshift(hpfData2{i}, -shiftHpf2);
end

maeGyro1 = zeros(num,1);
rmseGyro1 = zeros(num,1);
maeGyro2 = zeros(num,1);
rmseGyro2 = zeros(num,1);
maeHpf1 = zeros(num,1);
rmseHpf1 = zeros(num,1);
maeHpf2 = zeros(num,1);
rmseHpf2 = zeros(num,1);

for i = 1:num
    hpfData1{i} = hpfData1{i}(1001:4000,:);
    gyroData1{i} = gyroData1{i}(1001:4000,:);
    hpfData2{i} = hpfData2{i}(1001:4000,:);
    gyroData2{i} = gyroData2{i}(1001:4000,:);
end

for i = 2:num
    diffGyro1 = gyroData1{i} - gyroData1{1};
    maeGyro1(i) = mean(abs(diffGyro1));
    rmseGyro1(i) = sqrt(mean(diffGyro1.^2));

    diffHpf1 = hpfData1{i} - hpfData1{1};
    maeHpf1(i) = mean(abs(diffHpf1));
    rmseHpf1(i) = sqrt(mean(diffHpf1.^2));

    diffGyro2 = gyroData2{i} - gyroData2{1};
    maeGyro2(i) = mean(abs(diffGyro2));
    rmseGyro2(i) = sqrt(mean(diffGyro2.^2));

    diffHpf2 = hpfData2{i} - hpfData2{1};
    maeHpf2(i) = mean(abs(diffHpf2));
    rmseHpf2(i) = sqrt(mean(diffHpf2.^2));
end

rmseGyro = (rmseGyro1(6) + rmseGyro2(6)) / 2;
rmseHpf = (rmseHpf1(6) + rmseHpf2(6)) / 2;
rmse = (rmseGyro - rmseHpf) / rmseGyro * 100;
disp(rmse);


figTime = (0:3000-1) * 1/125;

hold off;
% plot(rmseGyro1,"-o","Color","#c93a40","LineWidth",2.0); hold on;
% plot(rmseHpf1,"-o","Color","#0074bf","LineWidth",2.0);
% plot(rmseGyro2,"--o","Color","#c93a40","LineWidth",2.0);
% plot(rmseHpf2,"--o","Color","#0074bf","LineWidth",2.0);
figData = [rmseGyro1(2:6) rmseGyro2(2:6) rmseHpf1(2:6) rmseHpf2(2:6)];
b = bar(figData);
% b(1).FaceColor = [201 58 64]/255;
% b(1).EdgeColor = [201 58 64]/255;
% b(2).FaceColor = [201 58 64]/255;
% b(2).EdgeColor = [201 58 64]/255;
% b(3).FaceColor = [0 116 191]/255;
% b(3).EdgeColor = [0 116 191]/255;
% b(4).FaceColor = [0 116 191]/255;
% b(4).EdgeColor = [0 116 191]/255;
% b(1).FaceAlpha = 0.5; 
% b(1).EdgeAlpha = 0.5; 
% b(3).FaceAlpha = 0.5;
% b(3).EdgeAlpha = 0.5;

b(1).FaceColor = [255 255 255]/255;
b(1).EdgeColor = [201 58 64]/255;

b(2).FaceColor = [201 58 64]/255;
b(2).EdgeColor = [201 58 64]/255;

b(3).FaceColor = [255 255 255]/255;
b(3).EdgeColor = [0 116 191]/255;

b(4).FaceColor = [0 116 191]/255;
b(4).EdgeColor = [0 116 191]/255;

b(1).LineWidth = 2;
b(2).LineWidth = 2;
b(3).LineWidth = 2;
b(4).LineWidth = 2;

legend({"Gyro1","Gyro2","Hpf1","Hpf2"},"FontSize",16,'Location', 'northeastoutside');
grid on;
ax = gca; 
ax.LineWidth = 1.5;
ax.FontSize = 12;
ylabel("RMSE","FontSize",20);
xlabel("replay count","FontSize",20);
ax.FontWeight = "bold";
exportgraphics(ax,'rmseCompare.pdf','ContentType','vector')

% hold off;
% plot(figTime,gyroData1{1},"Color","#c93a40","LineWidth",2.0); hold on;
% plot(figTime,gyroData1{3},"Color","#0074bf","LineWidth",2.0);
% plot(figTime,gyroData1{6},"Color","#56a764","LineWidth",2.0);
% legend("raw","2","5","FontSize",16);
% xlabel("time(s)","FontSize",20)
% ylabel("angle(rad)","FontSize",20)
% grid on;
% ax = gca; 
% ax.LineWidth = 1.5;
% ax.FontSize = 12;
% ax.FontWeight = "bold";
% exportgraphics(ax,'replayGyro1.pdf','ContentType','vector')
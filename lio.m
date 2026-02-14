[fn,pn] = uigetfile(".csv");
data = readmatrix(strcat(pn,fn));
[r,c] = size(data);
Fs = 50000/(5+3+8+16*3+16);
L = 0.42; 
%yoff = 0.9; 
%data = data-yoff; 
t = (0:1/Fs:(r-1)/Fs); 
f = (0:Fs/r:Fs-Fs/r);
N = 50; 
Fp = 5; 
Fh = 2; 
Rp = 0.00057565; 
Rst = 1E-5; 
eqnum = firceqrip(N, Fp/Fs*2, [Rp Rst], 'passedge'); 
eqnumh = firceqrip(N, Fh/Fs*2, [Rp Rst], 'high'); 
hpf = dsp.FIRFilter('Numerator', eqnumh); 
mm = 50; 
maco = 1/mm*ones(mm, 1); 
maf = dsp.FIRFilter('Numerator', maco'); 
lpf = dsp.FIRFilter('Numerator', eqnum); 
%dcblk = dsp.DCBlocker("NormalizedBandwidth", 0.01, "Order", 10); 
dcblk = dsp.DCBlocker('Algorithm','IIR','Order', 3);
cc = 8/8192*9.8;
phi = 0; %5/180*pi; %inclination in y axis
g = 0.0; 
gvec = [-sin(phi)*g; 0; cos(phi)*g]; 
ax = data(:,3)*cc/L; % x axis
ay = data(:,4)*cc/L; % time data
az = data(:,5)*cc; 
psi = [0. 0.0 0];
psimat = [1 psi(3) -psi(2); -psi(3) 1 psi(1); psi(2) -psi(1) 1]; 
aa = psimat*[ax'; ay'; az']; 
ax = -aa(1,:)'; 
ay = aa(2,:)'; 
az = aa(3,:)'; 
%ax = hpf(ax) ; 
%ay = hpf(ay); 
%ax = ax- 3.0; 
%
%ax(ax>0) = 0; 
ax = ax; %-mean(ax); %-mean(ax); %- mean(ax); 
ay = ay - mean(ay); 
%ax = dcblk(ax); 
%y = dcblk(ay); 
%axf = filtfilt(eqnum, 1, ax); 
%ayf = filtfilt(eqnum, 1, ay); 
%subplot(2,1,1); 
%plot(ax, ay); 
%axf = dcblk(axf); 
%ayf = dcblk(ayf); 
thf = integy2(ax, ay, az, gvec, Fs);
%yf = integy2(ay, Fs); 
%subplot(2,1,2); 
hold off; 
plot(t, thf); 
hold on; 
plot(t, hpf(thf)); % velocity  
%vf = lpf(vf); 
function yfint = integy(yf, Fs)
[r, ~] = size(yf); 
yfint = zeros(r, 1); 
s = 0; 
for i=1:r
    yfint(i) = s + yf(i)*1/Fs; 
    s = yfint(i); 
end
%yfint = trapz(1/Fs, yf); 
end
function yfint = integy2(mx, my, mz, gvec, Fs)
[r, ~] = size(mx); 
yfint = zeros(r, 1); 
cur = 0; 
prev = 0;
tic
for i=1:r-1
    %fprintf("%d  ", i); 
    yfint(i+1) = newtonsol(cur, prev, mx(i), my(i), mz(i), gvec, Fs);
    cur = yfint(i+1); 
    prev = yfint(i); 
end
toc
%yfint = trapz(1/Fs, yf); 
end
function th = newtonsol(cur, prev, mx, my, mz, gvec, Fs)
   th = cur; 
   ggvec = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 1] *gvec; 
   d = 1/Fs; 
   for i=1:10
       %r = (-(th-cur)*(th-cur)/d/d - mx + ggvec(1))*(-2*(th-cur)/d/d) ...
       %    + ((th-2*cur+prev)/d/d - my + ggvec(2))/d/d ; 
       r = (-(th-cur)*(th-cur)/d/d - mx)*(-2*(th-cur)/d/d) ...
           + ((th-2*cur+prev)/d/d - my)/d/d ; 
       if (abs(r) < 1E-3)
           break;
       else 
        fprintf("%d the=%f res = %f\n", i, th, abs(r)); 
       jac = (-2*(th-cur)/d/d) *(-2*(th-cur)/d/d) + (-(th-cur)*(th-cur)/d/d-mx)^2 *(-2/d/d) + 1/d/d/d/d; 
       th = th - r/jac; 
       end 
   end
end
function thListReturn = anglecalc(meas,range,fs,length,filt)
    gOff = 8192/range;  %1gの値
    dt = 1/fs;
    gw = 9.80665; %重力
    L = length;  %第1アーム長
    [thNext,thCurr,thPrev] = deal(0,0,0); %計算角度(Next:求める,Curr:今,Prev:一つ前)
    
    [measSize,~] = size(meas);
    thList = zeros(measSize,1);
    meas(:,1) = meas(:,1) + 174.46;
    meas(:,2) = meas(:,2) + 110;
    
    Rp = 0.00057565;
    Rst = 1e-3;
    lpTap = 400;
    lpNum = firceqrip(lpTap, 0.01/(fs/2),[Rp,Rst]);
    lpNum = lpNum/sum(lpNum); 
    lpf = dsp.FIRFilter("Numerator",lpNum);

    avTap = 300; 
    avNum = ones(avTap, 1)/avTap;
    avf = dsp.FIRFilter('Numerator', avNum');
    
    meas = meas / gOff * gw / L;
    
    %%  ループ ----------------------------------------------------------------
    for k = 1:measSize
        mx = meas(k,1);
        my = meas(k,2);
        mz = meas(k,3);
    
        count = 0;
        while count < 100
            count = count + 1;
            om = (thNext - thCurr) / dt;
            al = (thNext - 2*thCurr + thPrev) / dt/dt;
            lt = (-om*om - mx) * (-2*om/dt) + (al-my)/dt/dt;
            if abs(lt)< 1E-3
                break;
            else
                J = (-2*om/dt) * (-2*om/dt) + (-om*om - mx) * (-2/dt/dt) + 1/dt/dt/dt/dt; 
                thNext = thNext - lt / J;
            end 
        end
        thList(k,1) = thNext;
    
        thPrev = thCurr;
        thCurr = thNext;
    end
    
    thListLpf = lpf(thList);
    thListLpf = thList- [thListLpf(floor(lpTap/2)+1:end);zeros(floor(lpTap/2),1)]; 
    thListLpf = thListLpf';

    thListAvf = avf(thList);
    thListAvf = thList- [thListAvf(floor(avTap/2)+1:end);zeros(floor(avTap/2),1)]; 
    thListAvf = thListAvf';

    if filt == 0
        thListReturn = thListLpf;
    elseif filt == 1
        thListReturn = thListAvf;
    end
end

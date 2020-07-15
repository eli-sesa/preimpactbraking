function dataOut = cfc_filter(dataIn, filterOrder, tSample=10000)
    pkg load signal 
    nyquist = tSample/2; # hz Assume 10khz sampling default 
    cutoff = 10/6 * filterOrder; # convert filter class to cutoff freq
    
    wn = cutoff / nyquist; # [] Normalize for digital filter package
    [b, a] = butter(1, wn); # first order butterworth (analogous to simple RC filter)
    dataOut = filtfilt(b,a,dataIn);
    
endfunction
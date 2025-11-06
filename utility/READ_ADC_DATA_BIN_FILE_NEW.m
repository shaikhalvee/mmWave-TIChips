function ret_adc_data = READ_ADC_DATA_BIN_FILE_NEW(adc_data_bin_file, mmWave_device)
% Hardcoded read parameters:
numADCSamples = 256; numADCBits = 16; numRX = 4; isReal = 0;

fid = fopen(adc_data_bin_file,'r');
rawSamples = fread(fid, 'int16');  fclose(fid);
fileSize = size(rawSamples, 1);

% Bit-depth correction if not full 16-bit:
if numADCBits ~= 16
    l_max = 2^(numADCBits-1)-1;
    rawSamples(rawSamples > l_max) = rawSamples(rawSamples > l_max) - 2^numADCBits;
end

% Real data conversion
if isReal
    numChirps = fileSize/numADCSamples/numRX;

    LVDS = zeros(1, fileSize);
    LVDS = reshape(rawSamples, numADCSamples*numRX, numChirps);
    LVDS = LVDS.';
else
    % Complex IQ reconstruction (interleaved):
    numChirps = fileSize/2/numADCSamples/numRX;
    LVDS = zeros(1, fileSize/2);
    counter = 1;
    for i=1:4:numel(rawSamples)-1
        LVDS(counter)   = rawSamples(i)   + 1j*rawSamples(i+2);
        LVDS(counter+1) = rawSamples(i+1) + 1j*rawSamples(i+3);
        counter = counter + 2;
    end
    LVDS = reshape(LVDS, numADCSamples*numRX, numChirps).';
end


% Demultiplex per Rx channel:
% Meaning, the data is parsed according to the Rx channels
adcMatrix = zeros(numRX, numChirps*numADCSamples);
for ch = 1:numRX
    for k = 1:numChirps
        startIdx = (k-1)*numADCSamples + 1;
        adcMatrix(ch, startIdx : k*numADCSamples) = LVDS(k, (ch-1)*numADCSamples + 1 : ch*numADCSamples);
    end
end

ret_adc_data = adcMatrix;
end

clear all;

% parse results file
flag = 0;
i = 1;
fid = fopen('results.txt');
tline = fgetl(fid);
while ischar(tline)
    if(flag == 1)
        freq_hist{i} = tline;
        i = i + 1;
    end;
    
    pos = findstr(tline, 'temporal_frequency: ');
    if(pos ~= 0)
        temp_hist = tline;
    end;
    
    pos = findstr(tline, 'frequency_histogram: ');
    if(pos ~= 0)
        flag = 1;
    end
    
    pos = findstr(tline, 'frame_window: ');
    if(pos ~= 0)
        frame_window = tline;
    end
    
    tline = fgetl(fid);
end
fclose(fid);

t_h = strsplit(temp_hist, ' ');
f_w = strsplit(frame_window, ' ');
t_h = t_h(2:length(t_h));
f_w = f_w(2);

amp = 1;
for j = 1:length(t_h)-1
    duration = str2num(cell2mat(f_w));
    freq = str2num(cell2mat(t_h(j)));
    fs = 4*freq;                        % sampling frequency
    values = 0:1/fs:duration;
    a{j} = amp*sin(2*pi* freq*values);
end

% concat all the generated values and create the sound
b = cell2mat(a);
audiowrite('sound_a.wav', b, fs);
plot(b)
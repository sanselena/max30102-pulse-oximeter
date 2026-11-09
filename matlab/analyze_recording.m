% analyze_recording.m - first look at one recording from record_session.m.
%
% Prints lost samples and quality flags, plots the signals, computes the signal-to-noise ratio
% (Section 2.8: power at the heart rate and its harmonics vs the rest of the 0.5-5 Hz band)
% and lists R for every 10 s window, to put next to the reference oximeter readings.
% Uses only base MATLAB (no toolboxes).

file = "S01_rest_20261112_101500.csv";   % change to your file
snrSeconds = 60;                          % last 60 s of the recording, as in Table 10 step 7
fs = 100;

T = readtable(file);
t = T.t_ms / 1000;
fprintf("%s: %d rows, %.1f s\n", file, height(T), t(end) - t(1));

% Lost samples: the Uno's time axis jumps when the sensor's FIFO overflowed.
dt = diff(T.t_ms);
lost = sum(dt(dt > 10) / 10 - 1);
fprintf("Lost samples: %d\n", lost);

% Quality flags (bits): 1 no finger, 2 saturation, 4 low perfusion, 8 ambient light
names = ["no finger", "saturation", "low perfusion", "ambient light"];
for b = 0:3
    share = mean(bitand(T.flags, 2^b) > 0) * 100;
    fprintf("  %-14s %5.1f %% of samples\n", names(b + 1), share);
end

% Plots
figure("Name", file);
ax1 = subplot(4, 1, 1); plot(t, T.red, t, T.ir); ylabel("raw counts"); legend("red", "IR");
title("Raw signals (flat at 262143 = saturated)");
ax2 = subplot(4, 1, 2); plot(t, T.red_ac, t, T.ir_ac); ylabel("AC"); legend("red", "IR");
ax3 = subplot(4, 1, 3); plot(t, T.hr); ylabel("HR (bpm)");
ax4 = subplot(4, 1, 4); plot(t, T.spo2); ylabel("SpO_2 (%)"); xlabel("time (s)");
linkaxes([ax1 ax2 ax3 ax4], "x");

% Signal-to-noise ratio on the last snrSeconds of the IR signal
n = min(height(T), snrSeconds * fs);
x = T.ir(end - n + 1:end);
x = x - mean(x);
w = 0.5 - 0.5 * cos(2 * pi * (0:n - 1)' / (n - 1));   % Hann window
P = abs(fft(x .* w)).^2;
f = (0:n - 1)' * fs / n;
band = f >= 0.5 & f <= 5;
search = f >= 0.5 & f <= 3.5;                          % 30-210 bpm
[~, k] = max(P .* search);
f0 = f(k);
sig = false(n, 1);
for h = 1:floor(5 / f0)
    sig = sig | abs(f - h * f0) <= 0.1;
end
sig = sig & band;
snrDb = 10 * log10(sum(P(sig)) / sum(P(band & ~sig)));
hrLast = T.hr(end - n + 1:end);
hrFirmware = median(hrLast(hrLast > 0));                 % 0 means "no value yet"
fprintf("SNR (last %d s): %.1f dB, heart rate from the spectrum %.0f bpm, firmware median %.0f bpm\n", ...
    round(n / fs), snrDb, f0 * 60, hrFirmware);

% R and firmware values per 10 s window, for the calibration table
fprintf("\n  window      R     firmware SpO2  firmware HR\n");
for s0 = 0:10:floor(t(end)) - 10
    idx = t >= s0 & t < s0 + 10 & T.flags == 0;
    if nnz(idx) < 500, continue; end                   % less than half the window is clean
    acR = std(T.red_ac(idx)); acI = std(T.ir_ac(idx));    % RMS instead of peak-to-peak: same ratio
    R = (acR / mean(T.red(idx))) / (acI / mean(T.ir(idx)));
    fprintf("  %3d-%3d s  %6.3f   %6.1f         %5.0f\n", s0, s0 + 10, R, ...
        median(T.spo2(idx)), median(T.hr(idx)));
end

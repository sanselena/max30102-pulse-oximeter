% record_session.m - saves the Uno's CSV output to a file for offline analysis.
%
% Before running: upload oximeter_main to the Uno and CLOSE the Arduino Serial Monitor and
% Serial Plotter (only one program can use the COM port at a time).
% Opening the port restarts the Uno, so the recording starts from the sensor setup.
%
% Every 10 s the Command Window prints the Uno time: write the reference oximeter's
% SpO2 and pulse next to it (Section 2.6).

port = "COM5";             % Arduino IDE > Tools > Port shows the right one
durationS = 150;           % 150 s per test state, as in Section 2.6
subject = "S01";           % volunteer code, never the name
state = "rest";            % rest / light_activity / finger_movement / ambient_light

outFile = sprintf("%s_%s_%s.csv", subject, state, string(datetime("now", "Format", "yyyyMMdd_HHmmss")));

s = serialport(port, 115200, "Timeout", 5);
configureTerminator(s, "CR/LF");
flush(s);
% If the script stops with an error, type "clear s" to free the COM port.

fid = fopen(outFile, "w");
fprintf(fid, "t_ms,red,ir,red_ac,ir_ac,hr,spo2,flags\n");

fprintf("Waiting for the Uno to start...\n");
started = false;
nextMark = 0;
nRows = 0;
while true
    line = readline(s);
    if isempty(line)
        fprintf("No data for 5 s. Is the right port selected and the sketch uploaded?\n");
        break
    end
    line = strtrim(line);
    if startsWith(line, "#") || startsWith(line, "t_ms")
        fprintf("%s\n", line);      % messages from the Uno
        continue
    end
    v = str2double(split(line, ","));
    if numel(v) ~= 8 || any(isnan(v))
        continue                     % a line cut in half when the port opened
    end
    if ~started
        started = true;
        fprintf("Recording %d s to %s\n", durationS, outFile);
    end
    fprintf(fid, "%s\n", line);
    nRows = nRows + 1;

    tS = v(1) / 1000;
    if tS >= nextMark
        fprintf("t = %6.1f s   Uno HR %3d  SpO2 %5.1f  flags %d   -> write down the reference now\n", ...
            tS, v(6), v(7), v(8));
        nextMark = nextMark + 10;
    end
    if nRows >= durationS * 100
        break
    end
end

fclose(fid);
clear s
fprintf("Saved %d rows (%.1f s) to %s\n", nRows, nRows / 100, outFile);

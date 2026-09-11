%% FMCW MIMO RADAR — 2x2 software-sync host script (ESP32-C3 + USB-6366)
%
%  This is a 2 TX × 2 RX MIMO configuration using ONLY working antennas:
%    Active TX channels: 1 and 4   (TX2, TX3 are damaged — see sweep test results)
%    Active RX channels: 1 and 4   (RFIN2, RFIN3 antennas damaged)
%  Virtual array: 2 × 2 = 4 elements (vs 16 for full 4×4 array)
%  Once TX2/TX3 baluns are reflowed and RX2/RX4 antennas working,
%  switch back to the 4×4 version (NUM_TX = NUM_RX = 4).
%
%  Synchronization: software sync via USB-serial.
%    MATLAB sends 'C N' (set TX channel directly via SPI) → wait OK →
%    start DAQ ramp + capture → read N samples on 2 RX channels →
%    repeat for next TX → repeat for next frame.
%
%  Connections:
%    ESP32-C3 USB    → MATLAB serial (115200 baud)
%    USB-6366 AO 0   → VCO Vtune (chirp ramp)
%    USB-6366 AI 0   → IFOUT1 (RX1) — connect via BNC, FS/GS to GS for single-ended
%    USB-6366 AI 1   → IFOUT4 (RX4) — same
%    ESP32-C3 GPIOs  → ADAR2001 + ADAR2004 control via J1 connectors

clear; clc; close all;

USE_HARDWARE = false;
ESP32_PORT = 'COM3';
DAQ_NAME   = 'Dev1';

% --- Active channels (the only working antennas right now) ---
TX_CHANNELS = [1 4];           % active TX (skip damaged TX2, TX3)
RX_CHANNELS = [1 4];           % active RX (skip damaged RFIN2, RFIN3)
NUM_TX = length(TX_CHANNELS);  % 2
NUM_RX = length(RX_CHANNELS);  % 2

% --- AI channel mapping ---
% Which DAQ AI channel carries each RX. Default: sequential (AI 0, AI 1).
% Adjust if you wire IFOUT1/IFOUT4 to different BNCs on the DAQ.
RX_AI_MAP = [0 1];   % RX_CHANNELS(1) -> AI 0, RX_CHANNELS(2) -> AI 1

% --- Radar parameters ---
c       = 3e8;
fstart  = 16e9;
fstop   = 20e9;
BW      = fstop - fstart;
Tp      = 1e-3;
FS      = 1e6;
N       = round(Tp * FS);

V_START = 2.0;  V_STOP = 6.0;  % VCO tuning (calibrate!)

% --- Processing ---
zpad       = 8 * N / 2;
rr         = c / (2 * BW);
max_range  = rr * N / 2;
range_axis = linspace(0, max_range, zpad/2);
dbv = @(x) 20*log10(abs(x) + eps);

fprintf('=== 2 TX × 2 RX MIMO (4 virtual channels) ===\n');
fprintf('Active TX: [%s], Active RX: [%s]\n', ...
    sprintf('%d ', TX_CHANNELS), sprintf('%d ', RX_CHANNELS));
fprintf('BW: %.1f GHz | Resolution: %.2f cm | Max range: %.1f m\n', ...
    BW/1e9, rr*100, max_range);
fprintf('Software sync: ~%d ms per MIMO frame\n\n', ...
    round(NUM_TX * (Tp*1000 + 10)));

%% ============================================================
%  HARDWARE INIT
%  ============================================================
ard = []; dq_ao = []; dq_ai = [];

if USE_HARDWARE
    fprintf('Connecting to ESP32-C3 on %s...\n', ESP32_PORT);
    ard = serialport(ESP32_PORT, 115200, 'Timeout', 5);
    configureTerminator(ard, 'LF');
    pause(2);
    flush(ard);

    fprintf('Verifying SPI...\n');
    resp = esp32_cmd(ard, 'V');
    if contains(resp, 'FAIL')
        error('SPI verification failed: %s', resp);
    end
    fprintf('  %s\n', resp);

    fprintf('Initializing ADAR ICs...\n');
    resp = esp32_cmd(ard, 'I');
    fprintf('  %s\n', resp);

    % NOTE: We do NOT enable sequencers (no 'S' command).
    % We use SPI direct mode and select TX channel via 'C N' before each chirp.
    % This lets us pick any subset of channels (e.g., 1 and 4) easily.

    fprintf('Setting up USB-6366 (analog only, no SPI)...\n');

    dq_ao = daq('ni');
    addoutput(dq_ao, DAQ_NAME, 'ao0', 'Voltage');
    dq_ao.Rate = 100e3;
    ramp = linspace(V_START, V_STOP, round(dq_ao.Rate * Tp))';

    dq_ai = daq('ni');
    for k = 1:NUM_RX
        ai = addinput(dq_ai, DAQ_NAME, sprintf('ai%d', RX_AI_MAP(k)), 'Voltage');
        ai.TerminalConfig = 'Differential';
        ai.Range = [-1 1];
    end
    dq_ai.Rate = FS;
    fprintf('  AI mapping: ');
    for k = 1:NUM_RX
        fprintf('RX%d->AI%d ', RX_CHANNELS(k), RX_AI_MAP(k));
    end
    fprintf('\nHardware ready.\n\n');
else
    fprintf('SIMULATION MODE\n\n');
end

%% ============================================================
%  CAPTURE: One MIMO frame (software sync, SPI direct mode)
%  ============================================================
function frame_data = capture_frame(ard, dq_ao, dq_ai, ramp, Tp, N, ...
        NUM_TX, NUM_RX, TX_CHANNELS)
    frame_data = zeros(N, NUM_RX, NUM_TX);

    for tx = 1:NUM_TX
        % Set this TX channel directly via SPI (no state machine).
        % 'C N' writes register 0x045 with the appropriate channel mask.
        esp32_cmd(ard, sprintf('C %d', TX_CHANNELS(tx)));

        preload(dq_ao, ramp);
        start(dq_ai, 'Duration', seconds(Tp));
        start(dq_ao);

        pause(Tp + 0.002);

        data = read(dq_ai, N);
        frame_data(:, :, tx) = table2array(data);
        stop(dq_ao);
    end
end

%% ============================================================
%  SIMULATE: One MIMO frame, sparse array geometry
%  ============================================================
function frame_data = simulate_frame(targets, t_abs, N, FS, ...
        NUM_TX, NUM_RX, TX_CHANNELS, RX_CHANNELS, ...
        Tp, BW, fstart, fstop, c)
    frame_data = zeros(N, NUM_RX, NUM_TX);
    t_sample = (0:N-1)' / FS;
    lambda_c = c / ((fstart + fstop) / 2);
    d_tx_unit = 2 * lambda_c;
    d_rx_unit = lambda_c / 2;
    theta = 10 * pi / 180;

    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            tx_pos = (TX_CHANNELS(tx) - 1) * d_tx_unit;
            rx_pos = (RX_CHANNELS(rx) - 1) * d_rx_unit;
            sig = zeros(N, 1);
            for tgt = 1:size(targets, 1)
                R = targets(tgt, 1) + targets(tgt, 3) * t_abs;
                if R < 0.1 || R > 50, continue; end
                f_beat = 2 * R * BW / (c * Tp);
                phase_rt = 4 * pi * R * (fstart + fstop) / (2 * c);
                arr_ph = 2*pi/lambda_c * (tx_pos + rx_pos) * sin(theta);
                sig = sig + targets(tgt,2) * cos(2*pi*f_beat*t_sample + phase_rt + arr_ph);
            end
            frame_data(:, rx, tx) = sig + 0.003 * randn(N, 1);
        end
    end
end

%% ============================================================
%  TARGETS (simulation)
%  ============================================================
if ~USE_HARDWARE
    targets = [
        3.0,  0.10,   0.0;
        7.5,  0.06,   0.8;
        12.0, 0.04,  -0.3;
    ];
end

%% ============================================================
%  DATA COLLECTION
%  ============================================================
NUM_FRAMES = 100;
frame_time = NUM_TX * Tp;
time_axis  = (0:NUM_FRAMES-1) * frame_time;
sif_all    = zeros(N, NUM_RX, NUM_TX, NUM_FRAMES);

if USE_HARDWARE
    fprintf('Capturing %d MIMO frames (software sync, 2x2)...\n', NUM_FRAMES);
    tic;
    for frame = 1:NUM_FRAMES
        sif_all(:,:,:,frame) = capture_frame(ard, dq_ao, dq_ai, ...
            ramp, Tp, N, NUM_TX, NUM_RX, TX_CHANNELS);
        if mod(frame, 25) == 0
            fprintf('  Frame %d/%d (%.1f s elapsed)\n', frame, NUM_FRAMES, toc);
        end
    end
    fprintf('Capture done: %.1f s\n\n', toc);
else
    fprintf('Simulating %d frames...\n', NUM_FRAMES);
    tic;
    for frame = 1:NUM_FRAMES
        t_abs = (frame - 1) * frame_time;
        sif_all(:,:,:,frame) = simulate_frame(targets, t_abs, ...
            N, FS, NUM_TX, NUM_RX, TX_CHANNELS, RX_CHANNELS, ...
            Tp, BW, fstart, fstop, c);
    end
    fprintf('Simulation done: %.1f s\n\n', toc);
end

%% ============================================================
%  PROCESSING
%  ============================================================
fprintf('Processing...\n');

% RTI on TX1-RX1 (indices 1,1 in active arrays)
sif = squeeze(sif_all(:, 1, 1, :))';

figure('Name', 'Raw RTI', 'Position', [50 500 900 350]);
v_raw = dbv(ifft(sif, zpad, 2));
S_raw = v_raw(:, 1:zpad/2);
imagesc(range_axis, time_axis, S_raw - max(S_raw(:)), [-60 0]);
colormap('jet'); colorbar;
ylabel('Time (s)'); xlabel('Range (m)');
title(sprintf('Raw RTI — TX%d-RX%d', TX_CHANNELS(1), RX_CHANNELS(1)));
xlim([0 min(18, max_range)]);

sif2 = sif(2:end,:) - sif(1:end-1,:);
figure('Name', '2-Pulse Canceller', 'Position', [50 100 900 350]);
v2 = dbv(ifft(sif2, zpad, 2));
S2 = v2(:, 1:zpad/2);
imagesc(range_axis, time_axis(1:end-1), S2 - max(S2(:)), [-60 0]);
colormap('jet'); colorbar;
ylabel('Time (s)'); xlabel('Range (m)');
title('2-Pulse Canceller — moving targets only');
xlim([0 min(18, max_range)]);

% MIMO non-coherent sum over 4 virtual channels
mid = round(NUM_FRAMES / 2);
mimo_sum = zeros(zpad/2, 1);
for tx = 1:NUM_TX
    for rx = 1:NUM_RX
        sp = fft(sif_all(:, rx, tx, mid) .* hanning(N), zpad);
        mimo_sum = mimo_sum + abs(sp(1:zpad/2));
    end
end

figure('Name', 'MIMO Range', 'Position', [1000 300 700 400]);
plot(range_axis, dbv(mimo_sum) - max(dbv(mimo_sum)), 'b', 'LineWidth', 1.5);
xlabel('Range (m)'); ylabel('dB');
title(sprintf('MIMO non-coherent sum (%d virtual channels, frame %d)', ...
    NUM_TX*NUM_RX, mid));
xlim([0 min(18, max_range)]); ylim([-60 0]); grid on;

%% ============================================================
%  CLEANUP
%  ============================================================
if USE_HARDWARE
    fprintf('Shutting down...\n');
    esp32_cmd(ard, 'W 045 00');      % all PAs off
    clear dq_ao dq_ai ard;
    fprintf('Hardware released.\n');
end

fprintf('\n============================================================\n');
fprintf('DONE\n');
fprintf('============================================================\n');

%% ============================================================
%  HELPER
%  ============================================================
function resp = esp32_cmd(ard, cmd)
    flush(ard);
    writeline(ard, cmd);
    resp = '';
    for attempt = 1:100
        if ard.NumBytesAvailable > 0
            line = readline(ard);
            if contains(line, 'OK') || contains(line, 'FAIL') || contains(line, 'ERR')
                resp = char(line);
                return;
            end
            if strlength(line) > 0
                fprintf('  [ESP32] %s\n', line);
            end
        else
            pause(0.01);
        end
    end
    resp = 'TIMEOUT';
    warning('ESP32 timeout on command: %s', cmd);
end

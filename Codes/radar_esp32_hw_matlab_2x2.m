%% FMCW MIMO RADAR — 2x2 hardware-trigger host script (ESP32-C3 + USB-6366)
%
%  This is a 2 TX × 2 RX MIMO configuration using ONLY working antennas:
%    Active TX channels: 1 and 4   (TX2, TX3 are damaged)
%    Active RX channels: 1 and 4   (RFIN2, RFIN3 antennas damaged)
%  Virtual array: 2 × 2 = 4 elements
%
%  IMPORTANT — firmware compatibility note:
%  The current radar_esp32c3_hw_trig.ino firmware uses the chip's state
%  machine which cycles through TX1->TX2->TX3->TX4 on consecutive TxADV
%  pulses (4 chirps per frame). Two practical options for 2x2 MIMO:
%
%    Option A (no firmware changes — used by this script):
%      Capture all 4 chirps per frame, but in MATLAB only USE the
%      TX1 and TX4 chirps (chirps 1 and 4). Discard chirps 2, 3.
%      Frame rate effectively halves (~125 fps instead of 250 fps).
%      Total wasted: 50% of capture time on damaged channels.
%
%    Option B (requires firmware update):
%      Reprogram the firmware state machine to cycle TX1 -> TX4 only,
%      and reduce the loop count from 4 to 2 in doMIMOFrame().
%      Specifically change:
%        spiWrite(PIN_CS_TX, 0x019, 0x12);  -> 0x14   (S1=1, S2=4)
%        spiWrite(PIN_CS_TX, 0x01A, 0x34);  -> 0x00   (S3, S4 unused)
%        for (uint8_t ch = 0; ch < 4; ch++)  -> "ch < 2"
%      Frame rate: full 250 fps. No wasted captures.
%
%  This script implements Option A (no firmware changes needed).
%  When you're ready for Option B, set USE_2TX_FIRMWARE = true and
%  reflash with the updated firmware.

clear; clc; close all;
fprintf('============================================================\n');
fprintf('FMCW MIMO RADAR — 2x2 HW Trigger (ESP32-C3 + USB-6366)\n');
fprintf('============================================================\n\n');

USE_HARDWARE = false;
USE_2TX_FIRMWARE = true;    % updated firmware natively cycles only TX1->TX4 (2 chirps/frame)
ESP32_PORT = 'COM3';
DAQ_NAME   = 'Dev1';

% --- Active channels ---
TX_CHANNELS = [1 4];           % active TX
RX_CHANNELS = [1 4];           % active RX
NUM_TX = length(TX_CHANNELS);  % 2
NUM_RX = length(RX_CHANNELS);  % 2
RX_AI_MAP = [0 1];             % AI 0 -> RX1, AI 1 -> RX4

% Firmware fires CHIRPS_PER_FRAME chirps per frame regardless of how many
% we actually use. With unmodified firmware, this is 4 (TX1, TX2, TX3, TX4).
if USE_2TX_FIRMWARE
    CHIRPS_PER_FRAME = NUM_TX;     % 2
    USED_CHIRP_IDX   = 1:NUM_TX;   % use all of them
else
    CHIRPS_PER_FRAME = 4;          % firmware cycles all 4 TX states
    USED_CHIRP_IDX   = TX_CHANNELS;% only chirps 1 and 4 are usable
end

% --- Radar parameters ---
c       = 3e8;
fstart  = 16e9;
fstop   = 20e9;
BW      = fstop - fstart;
Tp      = 1e-3;
FS      = 1e6;
N       = round(Tp * FS);

V_START = 2.0;  V_STOP = 6.0;

zpad       = 8 * N / 2;
rr         = c / (2 * BW);
max_range  = rr * N / 2;
range_axis = linspace(0, max_range, zpad/2);
dbv = @(x) 20*log10(abs(x) + eps);

NUM_FRAMES = 100;
frame_time = CHIRPS_PER_FRAME * Tp;
time_axis  = (0:NUM_FRAMES-1) * frame_time;

fprintf('Active TX: [%s], Active RX: [%s]\n', ...
    sprintf('%d ', TX_CHANNELS), sprintf('%d ', RX_CHANNELS));
fprintf('BW: %.1f GHz | Res: %.2f cm | Max: %.1f m\n', BW/1e9, rr*100, max_range);
if USE_2TX_FIRMWARE
    fprintf('Firmware mode: 2-TX (%.0f fps theoretical)\n\n', 1/frame_time);
else
    fprintf('Firmware mode: 4-TX, using only chirps [%s] (%.0f fps effective)\n\n', ...
        sprintf('%d ', USED_CHIRP_IDX), 1/frame_time);
end

%% ============================================================
%  HARDWARE INIT
%  ============================================================
ard = []; dq_ao = []; dq_ai = [];

if USE_HARDWARE
    fprintf('Connecting to ESP32-C3...\n');
    ard = serialport(ESP32_PORT, 115200, 'Timeout', 5);
    configureTerminator(ard, 'LF');
    pause(2); flush(ard);

    resp = esp32_cmd(ard, 'V');
    if contains(resp, 'FAIL'), error('SPI failed: %s', resp); end
    fprintf('  %s\n', resp);

    resp = esp32_cmd(ard, 'I');
    fprintf('  %s\n', resp);

    % Enable sequencers (hw_trig firmware needs this)
    resp = esp32_cmd(ard, 'S');
    fprintf('  %s\n', resp);

    % Match firmware chirp duration
    resp = esp32_cmd(ard, sprintf('T %d', round(Tp * 1e6)));
    fprintf('  %s\n\n', resp);

    fprintf('Setting up USB-6366...\n');

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

    % *** PFI 0 TRIGGER CONFIGURATION ***
    % Uncomment the method matching your DAQ Toolbox version:
    %
    % --- Method 1: DataAcquisition interface (R2020a+) ---
    % addtrigger(dq_ai, 'Digital', 'StartTrigger', ...
    %            [DAQ_NAME '/PFI0'], 'RisingEdge');
    %
    % --- Method 2: Session-based interface (R2014a-R2019b) ---
    % addTriggerConnection(dq_ai, 'External', ...
    %            [DAQ_NAME '/PFI0'], 'StartTrigger');

    fprintf('  AO: continuous repeating ramp at %.0f kHz\n', dq_ao.Rate/1e3);
    fprintf('  AI: PFI 0 triggered, %d samples/chirp at %.0f kHz\n', N, dq_ai.Rate/1e3);
    fprintf('  Wire: ESP32-C3 GPIO 19 -> USB-6366 PFI 0\n\n');
    fprintf('Hardware ready.\n\n');
else
    fprintf('SIMULATION MODE\n\n');
end

%% ============================================================
%  CAPTURE: Hardware-triggered multi-frame
%  ============================================================
function sif_all = capture_hw_triggered(ard, dq_ao, dq_ai, ramp, ...
        Tp, N, NUM_TX, NUM_RX, NUM_FRAMES, ...
        CHIRPS_PER_FRAME, USED_CHIRP_IDX)
    %
    % Captures CHIRPS_PER_FRAME chirps per frame from the firmware,
    % but extracts only the chirps at indices USED_CHIRP_IDX (TX1, TX4).
    %
    total_chirps  = NUM_FRAMES * CHIRPS_PER_FRAME;
    sif_all = zeros(N, NUM_RX, NUM_TX, NUM_FRAMES);

    num_ramp_reps = ceil(total_chirps * 1.5);
    ao_waveform = repmat(ramp, num_ramp_reps, 1);
    preload(dq_ao, ao_waveform);
    start(dq_ao, 'RepeatOutput');

    total_samples = N * total_chirps;
    start(dq_ai, 'Duration', seconds(total_chirps * Tp * 2));

    if NUM_FRAMES == 1
        esp32_cmd(ard, 'F');
    else
        esp32_cmd(ard, sprintf('F %d', NUM_FRAMES));
    end

    expected_time = total_chirps * (Tp + 0.0002);
    pause(expected_time + 0.5);

    try
        all_data = read(dq_ai, total_samples);
        raw = table2array(all_data);
    catch
        all_data = read(dq_ai, 'all');
        raw = table2array(all_data);
        fprintf('  Captured %d samples (expected %d)\n', size(raw,1), total_samples);
    end

    stop(dq_ao);

    % Extract only the chirps we want (TX1, TX4 = chirps 1, 4 in unmodified firmware)
    for frame = 1:NUM_FRAMES
        for tx_idx = 1:NUM_TX
            chirp_in_frame = USED_CHIRP_IDX(tx_idx);
            global_chirp = (frame - 1) * CHIRPS_PER_FRAME + chirp_in_frame;
            row_start = (global_chirp - 1) * N + 1;
            row_end   = global_chirp * N;
            if row_end <= size(raw, 1)
                sif_all(:, :, tx_idx, frame) = raw(row_start:row_end, :);
            else
                fprintf('  Warning: insufficient data at frame %d TX %d\n', frame, tx_idx);
            end
        end
    end
end

%% ============================================================
%  SIMULATE
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
%  TARGETS
%  ============================================================
if ~USE_HARDWARE
    targets = [3.0, 0.10, 0.0; 7.5, 0.06, 0.8; 12.0, 0.04, -0.3];
end

%% ============================================================
%  DATA COLLECTION
%  ============================================================
sif_all = zeros(N, NUM_RX, NUM_TX, NUM_FRAMES);

if USE_HARDWARE
    fprintf('Capturing %d frames (HW triggered, 2x2 active)...\n', NUM_FRAMES);
    tic;
    sif_all = capture_hw_triggered(ard, dq_ao, dq_ai, ramp, ...
        Tp, N, NUM_TX, NUM_RX, NUM_FRAMES, ...
        CHIRPS_PER_FRAME, USED_CHIRP_IDX);
    fprintf('Done: %.1f s (%.0f fps actual)\n\n', toc, NUM_FRAMES/toc);
else
    fprintf('Simulating %d frames...\n', NUM_FRAMES);
    tic;
    for frame = 1:NUM_FRAMES
        t_abs = (frame - 1) * frame_time;
        sif_all(:,:,:,frame) = simulate_frame(targets, t_abs, ...
            N, FS, NUM_TX, NUM_RX, TX_CHANNELS, RX_CHANNELS, ...
            Tp, BW, fstart, fstop, c);
    end
    fprintf('Done: %.1f s\n\n', toc);
end

%% ============================================================
%  PROCESSING
%  ============================================================
fprintf('Processing...\n');

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
title('2-Pulse Canceller — moving only'); xlim([0 min(18, max_range)]);

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
title(sprintf('MIMO non-coherent sum (%d virtual ch, frame %d)', ...
    NUM_TX*NUM_RX, mid));
xlim([0 min(18, max_range)]); ylim([-60 0]); grid on;

%% ============================================================
%  CLEANUP
%  ============================================================
if USE_HARDWARE
    esp32_cmd(ard, 'D');
    esp32_cmd(ard, 'W 045 00');
    clear dq_ao dq_ai ard;
    fprintf('\nHardware released.\n');
end
fprintf('\n=== DONE ===\n');

%% ============================================================
%  HELPER
%  ============================================================
function resp = esp32_cmd(ard, cmd)
    flush(ard);
    writeline(ard, cmd);
    resp = '';
    for attempt = 1:200
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
    warning('ESP32 timeout on: %s', cmd);
end

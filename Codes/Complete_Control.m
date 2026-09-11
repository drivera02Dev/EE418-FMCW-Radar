%% FMCW MIMO RADAR — Complete System
%  Mode 1: OFFLINE  — capture N frames, then plot everything
%  Mode 2: REALTIME — scrolling waterfall RTI, updates live
%
%  Gregory Charvat's MIT algorithm adapted for 4x4 MIMO, 16-20 GHz

clear; clc; close all;
fprintf('============================================================\n');
fprintf('FMCW MIMO RADAR — Complete System\n');
fprintf('============================================================\n\n');

%% ============================================================
%  CONFIGURATION — EDIT THIS SECTION
%  ============================================================
USE_HARDWARE = false;      % true = USB-6356, false = simulation
REALTIME     = false;      % true = scrolling waterfall, false = offline
DEV_NAME     = 'Babar';

% Radar parameters
c       = 3e8;
fstart  = 16e9;
fstop   = 20e9;
BW      = fstop - fstart;  % 4 GHz
Tp      = 1e-3;            % 1 ms chirp
NUM_TX  = 4;
NUM_RX  = 4;
FS      = 1e6;             % 1 MS/s
N       = round(Tp * FS);  % 1000 samples per chirp

% Timing
frame_time = NUM_TX * Tp;  % 4 ms per MIMO frame

% Offline mode settings
NUM_FRAMES_OFFLINE = 2500; % 2500 frames x 4ms = 10 seconds

% Real-time mode settings
RT_WINDOW   = 5.0;         % show last 5 seconds in waterfall
RT_DURATION = 30.0;        % run for 30 seconds (Ctrl+C to stop early)

% Processing
zpad = 8 * N / 2;          % zero-pad factor

% Derived
rr        = c / (2 * BW);
max_range = rr * N / 2;
range_axis = linspace(0, max_range, zpad/2);

% VCO ramp (calibrate for your HMC586)
V_START = 2.0;
V_STOP  = 6.0;

% Pin definitions
pins.SCLK=0; pins.SDIO=1; pins.CS_TX=2; pins.CS_RX=3;
pins.MADV_TX=4; pins.MRST_TX=5; pins.TxADV=6; pins.TxRST=7;

% Multiplier settings for 16-20 GHz
MULT_EN   = hex2dec('6E');
MULT_PASS = hex2dec('9F');

dbv = @(x) 20*log10(abs(x) + eps);

fprintf('Bandwidth:        %.1f GHz\n', BW/1e9);
fprintf('Range resolution: %.2f cm\n', rr*100);
fprintf('Max range:        %.1f m\n', max_range);
fprintf('Frame rate:       %.0f MIMO frames/s\n', 1/frame_time);
if REALTIME
    fprintf('Mode:             REAL-TIME (%.0f s window, %.0f s duration)\n', ...
        RT_WINDOW, RT_DURATION);
else
    fprintf('Mode:             OFFLINE (%d frames = %.1f s)\n', ...
        NUM_FRAMES_OFFLINE, NUM_FRAMES_OFFLINE * frame_time);
end
fprintf('\n');


%% ============================================================
%  HARDWARE INITIALIZATION
%  ============================================================
dq_dio = []; dq_ao = []; dq_ai = []; dq_sdo = [];
ps = [0 0 1 1 0 0 0 0];  % CS high

if USE_HARDWARE
    fprintf('Initializing USB-6356...\n');
    
    % Digital Output: P0.0–P0.7 (SPI + state machine)
    dq_dio = daq('ni');
    for i = 0:7
        addoutput(dq_dio, DEV_NAME, sprintf('port0/line%d',i), 'Digital');
    end
    write(dq_dio, ps);
    
    % Digital Input: P1.0 = SDO (shared from both ADAR ICs)
    % Wire: ADAR2001 Pin15 (SDO) + ADAR2004 Pin24 (SDO) → P1.0
    dq_sdo = daq('ni');
    addinput(dq_sdo, DEV_NAME, 'port1/line0', 'Digital');
    
    % Analog Output (VCO ramp)
    dq_ao = daq('ni');
    addoutput(dq_ao, DEV_NAME, 'ao0', 'Voltage');
    dq_ao.Rate = 100e3;
    ramp = linspace(V_START, V_STOP, round(dq_ao.Rate * Tp))';
    
    % Analog Input (4 diff IF channels)
    dq_ai = daq('ni');
    for ch = 0:3
        ai = addinput(dq_ai, DEV_NAME, sprintf('ai%d',ch), 'Voltage');
        ai.TerminalConfig = 'Differential';
        ai.Range = [-1 1];
    end
    dq_ai.Rate = FS;
    
    % --- SPI Verification: scratch-pad write → read back → compare ---
    fprintf('Verifying SPI link...\n');
    % Write scratch pads
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('00A'), hex2dec('A5'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('00A'), hex2dec('5A'), true);
    % Read back
    [ps, rd_tx] = spi_rd(dq_dio, dq_sdo, ps, pins, pins.CS_TX, hex2dec('00A'), true);
    [ps, rd_rx] = spi_rd(dq_dio, dq_sdo, ps, pins, pins.CS_RX, hex2dec('00A'), true);
    % Verify
    if rd_tx == hex2dec('A5')
        fprintf('  TX SPI PASS: wrote 0xA5, read 0x%02X ✓\n', rd_tx);
    else
        fprintf('  TX SPI FAIL: wrote 0xA5, read 0x%02X ✗\n', rd_tx);
        error('ADAR2001 SPI failed! Check wiring, level shifters, power.');
    end
    if rd_rx == hex2dec('5A')
        fprintf('  RX SPI PASS: wrote 0x5A, read 0x%02X ✓\n', rd_rx);
    else
        fprintf('  RX SPI FAIL: wrote 0x5A, read 0x%02X ✗\n', rd_rx);
        error('ADAR2004 SPI failed! Check wiring, level shifters, power.');
    end
    fprintf('  SPI verification complete.\n\n');
    
    % --- SPI: Initialize ADAR2001 ---
    fprintf('Initializing ADAR2001 (TX)...\n');
    % Soft reset
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('000'), hex2dec('81'), true); pause(0.01);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('000'), hex2dec('18'), true);
    % Power on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('010'), hex2dec('01'), true);
    % Bias (ADAR2001 defaults from Table 9)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('011'), hex2dec('BB'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('012'), hex2dec('0B'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('013'), hex2dec('75'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('014'), hex2dec('B5'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('015'), hex2dec('0C'), true);
    % SPI-direct multiplier config (used before sequencers enabled)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('047'), MULT_EN, true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('048'), MULT_PASS, true);
    
    % --- State machine MODES (must be set BEFORE enabling sequencers) ---
    % Multiplier Mode 0 = sleep
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('070'), hex2dec('00'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('071'), hex2dec('00'), true);
    % Multiplier Mode 1 = 16-20 GHz (mid band active, Table 7)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('072'), MULT_EN, true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('073'), MULT_PASS, true);
    % TX Mode 0 = all sleep
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('050'), hex2dec('00'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('051'), hex2dec('00'), true);
    % TX Mode 1 = CH1 active, splitters on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('052'), hex2dec('C0'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('053'), hex2dec('07'), true);
    % TX Mode 2 = CH2 active, splitters on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('054'), hex2dec('30'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('055'), hex2dec('07'), true);
    % TX Mode 3 = CH3 active, splitters on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('056'), hex2dec('0C'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('057'), hex2dec('07'), true);
    % TX Mode 4 = CH4 active, splitters on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('058'), hex2dec('03'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('059'), hex2dec('07'), true);
    
    % --- State assignments ---
    % Mult: State 1 → Mode 1 (16-20 GHz)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('03C'), hex2dec('10'), true);
    % TX: State 1→Mode 1(CH1), State 2→Mode 2(CH2)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('019'), hex2dec('12'), true);
    % TX: State 3→Mode 3(CH3), State 4→Mode 4(CH4)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('01A'), hex2dec('34'), true);
    
    % --- Enable sequencers (AFTER modes and states are configured) ---
    % 0x018: MULT_SEQ_EN=1, latch bypass=1, depth=0 (1 state in loop)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('018'), hex2dec('90'), true);
    % 0x016: TX_SEQ_EN=1, latch bypass=1
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('016'), hex2dec('90'), true);
    % 0x017: TX_STATES=3 (4 states in loop: CH1→CH2→CH3→CH4→repeat)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('017'), hex2dec('03'), true);
    
    % --- SPI: Initialize ADAR2004 ---
    %  NOTE: ADAR2004 has DIFFERENT bias registers than ADAR2001!
    %  All values from ADAR2004 datasheet Rev.A, Table 9/10.
    fprintf('Initializing ADAR2004 (RX)...\n');
    % Soft reset
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('000'), hex2dec('81'), true); pause(0.01);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('000'), hex2dec('18'), true);
    % Power on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('010'), hex2dec('01'), true);
    % ADAR2004 bias registers (NOT the same as ADAR2001!)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('011'), hex2dec('55'), true);  % mult bias
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('012'), hex2dec('07'), true);  % mult high bias
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('013'), hex2dec('78'), true);  % LO amp bias
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('014'), hex2dec('7A'), true);  % splitter bias
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('015'), hex2dec('2A'), true);  % LNA+mixer bias
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('016'), hex2dec('C0'), true);  % IF amp bias [7:4]
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('017'), hex2dec('04'), true);  % IF common-mode
    % Disable both sequencers (SPI-only control for RX)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('018'), hex2dec('00'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('019'), hex2dec('00'), true);
    % LO multiplier: 0xEE = mid band active, BPF low (Table 7, 4-5 GHz)
    %   (was 0xED — invalid: high band ACT=1 with RDY=0)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('02F'), hex2dec('EE'), true);
    % RX enable: 0xEF = LNA+Mixer+IFAMP on, all 4 channels on
    %   (was 0x6F — LNA disabled! No signal would reach mixers)
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('02B'), hex2dec('EF'), true);
    % Splitters all on
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('02E'), hex2dec('07'), true);
    % Gain: max on all channels
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('02C'), hex2dec('77'), true);
    ps = spi_wr(dq_dio, ps, pins, pins.CS_RX, hex2dec('02D'), hex2dec('77'), true);
    
    % Reset state machines
    ps = pulse_line(dq_dio, ps, pins.MRST_TX, true);
    ps = pulse_line(dq_dio, ps, pins.TxRST, true);
    pause(0.001);
    ps = pulse_line(dq_dio, ps, pins.MADV_TX, true);
    pause(0.0001);
    
    fprintf('Hardware ready.\n\n');
else
    fprintf('SIMULATION MODE\n\n');
end


%% ============================================================
%  FUNCTION: Capture one MIMO frame (4 TX sweeps, all RX)
%  ============================================================
% Returns: [N x NUM_RX x NUM_TX] array of IF samples

    function frame_data = capture_one_frame(dq_dio, dq_ao, dq_ai, ...
            ps, pins, ramp, Tp, N, NUM_TX, NUM_RX, use_hw)
        frame_data = zeros(N, NUM_RX, NUM_TX);
        if use_hw
            for tx = 1:NUM_TX
                ps = pulse_line(dq_dio, ps, pins.TxADV, true);
                pause(50e-6);
                preload(dq_ao, ramp);
                start(dq_ai, 'Duration', seconds(Tp));
                start(dq_ao);
                pause(Tp + 0.002);
                data = read(dq_ai, N);
                frame_data(:, :, tx) = table2array(data);
                stop(dq_ao);
            end
        end
    end

%% ============================================================
%  FUNCTION: Simulate one MIMO frame
%  ============================================================

    function frame_data = simulate_one_frame(targets, t_abs, ...
            N, FS, NUM_TX, NUM_RX, Tp, BW, fstart, fstop, c)
        frame_data = zeros(N, NUM_RX, NUM_TX);
        t_sample = (0:N-1)' / FS;
        lambda_c = c / ((fstart + fstop) / 2);
        
        for tx = 1:NUM_TX
            for rx = 1:NUM_RX
                sig = zeros(N, 1);
                for tgt = 1:size(targets, 1)
                    R = targets(tgt, 1) + targets(tgt, 3) * t_abs;
                    if R < 0.1 || R > 50, continue; end
                    amp = targets(tgt, 2);
                    f_beat = 2 * R * BW / (c * Tp);
                    phase_rt = 4 * pi * R * (fstart + fstop) / (2 * c);
                    d_tx = 2 * lambda_c; d_rx = lambda_c / 2;
                    theta = 10 * pi / 180;
                    arr_ph = 2*pi/lambda_c * ((tx-1)*d_tx + (rx-1)*d_rx) * sin(theta);
                    sig = sig + amp * cos(2*pi*f_beat*t_sample + phase_rt + arr_ph);
                end
                sig = sig + 0.003 * randn(N, 1);
                frame_data(:, rx, tx) = sig;
            end
        end
    end


%% ============================================================
%  SIMULATION TARGETS
%  ============================================================
if ~USE_HARDWARE
    % [range_m, amplitude, velocity_m/s]
    targets = [
        3.0,  0.10,   0.0;    % Stationary wall
        5.0,  0.08,   1.5;    % Person walking away
        8.0,  0.06,   0.0;    % Stationary object
       12.0,  0.04,  -1.0;    % Person approaching
    ];
    fprintf('Simulated targets:\n');
    for i = 1:size(targets,1)
        if targets(i,3)==0
            fprintf('  %.1f m — STATIONARY\n', targets(i,1));
        else
            fprintf('  %.1f m — MOVING %.1f m/s\n', targets(i,1), targets(i,3));
        end
    end
    fprintf('\n');
end


%% ============================================================
%  MODE SELECTION
%  ============================================================

if REALTIME
    %% ========================================================
    %  REAL-TIME SCROLLING WATERFALL
    %  ========================================================
    fprintf('========== REAL-TIME MODE ==========\n');
    fprintf('Running for %.0f s. Close figure or Ctrl+C to stop.\n\n', RT_DURATION);
    
    rt_frames = round(RT_WINDOW / frame_time);  % frames in display window
    
    % Circular buffer for RTI data
    rti_buffer = zeros(rt_frames, zpad/2);     % raw RTI
    rti_cancel_buf = zeros(rt_frames, zpad/2); % 2-pulse cancelled
    time_buffer = zeros(rt_frames, 1);
    prev_chirp = zeros(1, N);  % for 2-pulse canceller
    
    % Create figure with two subplots
    fig_rt = figure('Name', 'REAL-TIME RADAR', 'Position', [100 100 1200 700]);
    
    % Top: raw RTI
    ax1 = subplot(2,1,1);
    h_img1 = imagesc(ax1, range_axis, zeros(rt_frames,1), rti_buffer, [-50 0]);
    colormap(ax1, 'jet'); cb1 = colorbar(ax1);
    ylabel(ax1, 'Time (s)'); xlabel(ax1, 'Range (m)');
    title(ax1, 'Real-time RTI — all targets');
    xlim(ax1, [0 min(18, max_range)]);
    
    % Bottom: 2-pulse cancelled
    ax2 = subplot(2,1,2);
    h_img2 = imagesc(ax2, range_axis, zeros(rt_frames,1), rti_cancel_buf, [-50 0]);
    colormap(ax2, 'jet'); cb2 = colorbar(ax2);
    ylabel(ax2, 'Time (s)'); xlabel(ax2, 'Range (m)');
    title(ax2, 'Real-time RTI — 2-pulse canceller (moving only)');
    xlim(ax2, [0 min(18, max_range)]);
    
    drawnow;
    
    frame_count = 0;
    t_start = tic;
    
    while toc(t_start) < RT_DURATION && isvalid(fig_rt)
        frame_count = frame_count + 1;
        t_now = frame_count * frame_time;
        
        % Capture or simulate one frame
        if USE_HARDWARE
            fd = capture_one_frame(dq_dio, dq_ao, dq_ai, ...
                ps, pins, ramp, Tp, N, NUM_TX, NUM_RX, true);
        else
            fd = simulate_one_frame(targets, t_now, ...
                N, FS, NUM_TX, NUM_RX, Tp, BW, fstart, fstop, c);
        end
        
        % Process: sum all virtual channels for this frame
        chirp_sum = zeros(1, N);
        for tx = 1:NUM_TX
            for rx = 1:NUM_RX
                chirp_sum = chirp_sum + fd(:, rx, tx)';
            end
        end
        
        % Range profile (raw)
        spec = fft(chirp_sum .* hanning(N)', zpad);
        rp = abs(spec(1:zpad/2));
        
        % 2-pulse canceller
        diff_chirp = chirp_sum - prev_chirp;
        spec_c = fft(diff_chirp .* hanning(N)', zpad);
        rp_c = abs(spec_c(1:zpad/2));
        prev_chirp = chirp_sum;
        
        % Shift buffer up (scroll), add new frame at bottom
        rti_buffer = circshift(rti_buffer, -1, 1);
        rti_buffer(end, :) = dbv(rp);
        
        rti_cancel_buf = circshift(rti_cancel_buf, -1, 1);
        rti_cancel_buf(end, :) = dbv(rp_c);
        
        time_buffer = circshift(time_buffer, -1);
        time_buffer(end) = t_now;
        
        % Update plots every 10 frames (for speed)
        if mod(frame_count, 10) == 0
            m1 = max(rti_buffer(:));
            set(h_img1, 'CData', rti_buffer - m1, 'YData', time_buffer);
            set(ax1, 'YLim', [time_buffer(1) time_buffer(end)]);
            set(ax1, 'YDir', 'reverse');
            
            m2 = max(rti_cancel_buf(:));
            set(h_img2, 'CData', rti_cancel_buf - m2, 'YData', time_buffer);
            set(ax2, 'YLim', [time_buffer(1) time_buffer(end)]);
            set(ax2, 'YDir', 'reverse');
            
            title(ax1, sprintf('Real-time RTI — all targets (t = %.1f s)', t_now));
            title(ax2, sprintf('Real-time RTI — moving only (t = %.1f s)', t_now));
            drawnow limitrate;
        end
        
        % Pace simulation to roughly real-time
        if ~USE_HARDWARE
            pause(0.001);  % small pause so figure updates
        end
    end
    
    fprintf('Real-time capture stopped at %.1f s (%d frames).\n', ...
        frame_count * frame_time, frame_count);
    
    
else
    %% ========================================================
    %  OFFLINE MODE — Capture all, then plot
    %  ========================================================
    NUM_FRAMES = NUM_FRAMES_OFFLINE;
    total_time = NUM_FRAMES * frame_time;
    time_axis = (0:NUM_FRAMES-1) * frame_time;
    
    fprintf('========== OFFLINE MODE (%.1f s) ==========\n', total_time);
    
    % Capture/simulate all frames
    sif_all = zeros(N, NUM_RX, NUM_TX, NUM_FRAMES);
    
    if USE_HARDWARE
        fprintf('Capturing %d frames...\n', NUM_FRAMES);
        tic;
        for frame = 1:NUM_FRAMES
            sif_all(:,:,:,frame) = capture_one_frame(dq_dio, dq_ao, dq_ai, ...
                ps, pins, ramp, Tp, N, NUM_TX, NUM_RX, true);
            if mod(frame, 250) == 0
                fprintf('  Frame %d/%d\n', frame, NUM_FRAMES);
            end
        end
        fprintf('Capture done: %.1f s\n\n', toc);
    else
        fprintf('Simulating %d frames...\n', NUM_FRAMES);
        tic;
        for frame = 1:NUM_FRAMES
            t_abs = (frame - 1) * frame_time;
            sif_all(:,:,:,frame) = simulate_one_frame(targets, t_abs, ...
                N, FS, NUM_TX, NUM_RX, Tp, BW, fstart, fstop, c);
        end
        fprintf('Simulation done: %.1f s\n\n', toc);
    end
    
    
    %% --- Single-channel RTI (TX1-RX1, like Gregory's code) ---
    fprintf('Processing RTI (TX1-RX1)...\n');
    sif = squeeze(sif_all(:, 1, 1, :))';  % [frames x samples]
    
    % FIGURE 1: Raw RTI — NO subtraction
    figure('Name', 'Fig1: Raw RTI', 'Position', [50 600 900 400]);
    v_raw = dbv(ifft(sif, zpad, 2));
    S_raw = v_raw(:, 1:zpad/2);
    m_raw = max(S_raw(:));
    imagesc(range_axis, time_axis, S_raw - m_raw, [-60 0]);
    colormap('jet'); colorbar;
    ylabel('Time (s)'); xlabel('Range (m)');
    title('Fig 1: Raw RTI — all targets (vertical=stationary, diagonal=moving)');
    xlim([0 min(18, max_range)]);
    
    % FIGURE 2: Mean subtracted (Gregory's "without clutter rejection")
    sif_ms = sif;
    ave = mean(sif_ms, 1);
    for ii = 1:size(sif_ms, 1)
        sif_ms(ii,:) = sif_ms(ii,:) - ave;
    end
    
    figure('Name', 'Fig2: Mean Subtracted', 'Position', [50 350 900 400]);
    v_ms = dbv(ifft(sif_ms, zpad, 2));
    S_ms = v_ms(:, 1:zpad/2);
    m_ms = max(S_ms(:));
    imagesc(range_axis, time_axis, S_ms - m_ms, [-60 0]);
    colormap('jet'); colorbar;
    ylabel('Time (s)'); xlabel('Range (m)');
    title('Fig 2: Mean subtracted — moving targets only (diagonal)');
    xlim([0 min(18, max_range)]);
    
    % FIGURE 3: 2-pulse canceller (Gregory's "with clutter rejection")
    sif2 = sif(2:end,:) - sif(1:end-1,:);
    
    figure('Name', 'Fig3: 2-Pulse Canceller', 'Position', [50 100 900 400]);
    v2 = dbv(ifft(sif2, zpad, 2));
    S2 = v2(:, 1:zpad/2);
    m2 = max(S2(:));
    imagesc(range_axis, time_axis(1:end-1), S2 - m2, [-60 0]);
    colormap('jet'); colorbar;
    ylabel('Time (s)'); xlabel('Range (m)');
    title('Fig 3: 2-pulse canceller — moving targets only (diagonal)');
    xlim([0 min(18, max_range)]);
    
    
    %% --- MIMO Processing ---
    fprintf('Processing MIMO...\n');
    
    % FIGURE 4: MIMO range profiles (middle frame)
    frame_mid = round(NUM_FRAMES / 2);
    range_data_mid = zeros(zpad/2, NUM_RX, NUM_TX);
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            sig = sif_all(:, rx, tx, frame_mid);
            spec = fft(sig .* hanning(N), zpad);
            range_data_mid(:, rx, tx) = abs(spec(1:zpad/2));
        end
    end
    
    figure('Name', 'Fig4: MIMO Range', 'Position', [1000 450 800 500]);
    subplot(2,1,1); hold on;
    colors = lines(16); idx = 0;
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            idx = idx + 1;
            rp = range_data_mid(:, rx, tx);
            plot(range_axis, dbv(rp) - max(dbv(rp)), ...
                'Color', [colors(idx,:) 0.3], 'LineWidth', 0.5);
        end
    end
    xlabel('Range (m)'); ylabel('dB');
    title(sprintf('All 16 channels (frame %d)', frame_mid));
    xlim([0 min(18, max_range)]); ylim([-60 0]); grid on; hold off;
    
    subplot(2,1,2);
    coherent = sum(range_data_mid, [2 3]);  % sum over RX and TX dims
    coherent = coherent(:);
    plot(range_axis, dbv(coherent) - max(dbv(coherent)), 'b', 'LineWidth', 1.5);
    xlabel('Range (m)'); ylabel('dB');
    title(sprintf('Non-coherent sum (16 ch, %.1f dB gain)', 10*log10(NUM_TX*NUM_RX)));
    xlim([0 min(18, max_range)]); ylim([-60 0]); grid on;
    
    
    % FIGURE 5: MIMO RTI raw (all channels combined)
    fprintf('Computing MIMO RTI...\n');
    rti_mimo = zeros(NUM_FRAMES, zpad/2);
    for frame = 1:NUM_FRAMES
        for tx = 1:NUM_TX
            for rx = 1:NUM_RX
                sig = sif_all(:, rx, tx, frame);
                spec = fft(sig .* hanning(N), zpad);
                rti_mimo(frame, :) = rti_mimo(frame, :) + abs(spec(1:zpad/2))';
            end
        end
    end
    
    figure('Name', 'Fig5: MIMO RTI Raw', 'Position', [1000 250 800 400]);
    rti_db = dbv(rti_mimo);
    m_rti = max(rti_db(:));
    imagesc(range_axis, time_axis, rti_db - m_rti, [-50 0]);
    colormap('jet'); colorbar;
    ylabel('Time (s)'); xlabel('Range (m)');
    title('Fig 5: MIMO RTI — all 16 channels (all targets)');
    xlim([0 min(18, max_range)]);
    
    
    % FIGURE 6: MIMO RTI with 2-pulse canceller (FIXED)
    fprintf('Computing MIMO RTI with canceller...\n');
    rti_cancel = zeros(NUM_FRAMES - 1, zpad/2);
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            sif_ch = squeeze(sif_all(:, rx, tx, :))';  % [frames x N]
            sif_c = sif_ch(2:end,:) - sif_ch(1:end-1,:);  % 2-pulse cancel
            v_c = abs(ifft(sif_c, zpad, 2));  % [frames-1 x zpad]
            rti_cancel = rti_cancel + v_c(:, 1:zpad/2);  % FIXED: take first half
        end
    end
    
    figure('Name', 'Fig6: MIMO RTI Canceller', 'Position', [1000 50 800 400]);
    rti_c_db = dbv(rti_cancel);
    m_c = max(rti_c_db(:));
    imagesc(range_axis, time_axis(1:end-1), rti_c_db - m_c, [-50 0]);
    colormap('jet'); colorbar;
    ylabel('Time (s)'); xlabel('Range (m)');
    title('Fig 6: MIMO RTI — 2-pulse canceller (moving only)');
    xlim([0 min(18, max_range)]);
    
    
    %% --- Target Detection ---
    fprintf('\n========== Target Detection ==========\n');
    [pks, locs] = findpeaks(dbv(coherent) - max(dbv(coherent)), ...
        'MinPeakHeight', -30, ...
        'MinPeakDistance', round(0.3 / (range_axis(2) - range_axis(1))));
    fprintf('Detected %d targets:\n', length(locs));
    for ii = 1:length(locs)
        fprintf('  Target %d: %.2f m (%.1f dB)\n', ii, range_axis(locs(ii)), pks(ii));
    end
    
    %% --- Save data ---
    % Uncomment to save:
    % save('radar_data.mat', 'sif_all', 'FS', 'N', 'BW', 'Tp', ...
    %      'fstart', 'fstop', 'NUM_TX', 'NUM_RX', 'NUM_FRAMES', 'targets');
end


%% Cleanup
if USE_HARDWARE
    ps = spi_wr(dq_dio, ps, pins, pins.CS_TX, hex2dec('045'), hex2dec('00'), true);
    clear dq_dio dq_ao dq_ai dq_sdo;
    fprintf('\nHardware released.\n');
end

fprintf('\n============================================================\n');
fprintf('DONE\n');
fprintf('============================================================\n');
if ~REALTIME
    fprintf('  Fig 1: Raw RTI (all targets)\n');
    fprintf('  Fig 2: Mean subtracted (moving only)\n');
    fprintf('  Fig 3: 2-pulse canceller (moving only)\n');
    fprintf('  Fig 4: MIMO range profiles\n');
    fprintf('  Fig 5: MIMO RTI raw (16 ch)\n');
    fprintf('  Fig 6: MIMO RTI 2-pulse canceller (16 ch)\n');
end


%% ================================================================
%  LOCAL FUNCTIONS
%  ================================================================

function ps = set_pin(dq, ps, pin, val, hw)
    ps(pin+1) = val;
    if hw, write(dq, ps); end
end

function ps = spi_wr(dq, ps, pins, cs, addr, data, hw)
    % 24-bit SPI write: [R/W(1)][ADDR(15)][DATA(8)], MSB first
    %   Bit 23   = 0 (write)
    %   Bit 22-8 = A14..A0 (15-bit register address)
    %   Bit 7-0  = D7..D0  (8-bit data)
    word = bitor(bitshift(bitand(addr, hex2dec('7FFF')), 8), ...
                 bitand(data, hex2dec('FF')));
    ps = set_pin(dq, ps, cs, 0, hw);
    for i = 23:-1:0
        b = bitand(bitshift(word, -i), 1);
        ps = set_pin(dq, ps, pins.SDIO, b, hw);
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);
    end
    ps = set_pin(dq, ps, cs, 1, hw);
end

function ps = pulse_line(dq, ps, pin, hw)
    ps = set_pin(dq, ps, pin, 1, hw);
    if hw, pause(1e-6); end
    ps = set_pin(dq, ps, pin, 0, hw);
end

function [ps, data_out] = spi_rd(dq, dq_in, ps, pins, cs, addr, hw)
    % 24-bit SPI read: [R/W(1)][ADDR(15)][DATA(8)], MSB first
    %   Bit 23   = 1 (read)
    %   Bit 22-8 = A14..A0 (15-bit register address)
    %   Bit 7-0  = IC drives SDO with register contents
    %
    % 4-wire mode (default): data returned on SDO pin during last 8 clocks.
    % dq_in = separate input DAQ session reading P1.0 (SDO line).
    
    instr = bitor(bitshift(1, 23), ...
                  bitshift(bitand(addr, hex2dec('7FFF')), 8));
    
    data_out = 0;
    ps = set_pin(dq, ps, cs, 0, hw);
    
    % Instruction phase: 16 clocks (R/W + A14..A0)
    for i = 23:-1:8
        b = bitand(bitshift(instr, -i), 1);
        ps = set_pin(dq, ps, pins.SDIO, b, hw);
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);
    end
    
    % Data phase: 8 clocks — sample SDO on rising edge
    ps = set_pin(dq, ps, pins.SDIO, 0, hw);
    for i = 7:-1:0
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);
        if hw
            sdo_val = read(dq_in);
            if istable(sdo_val)
                sdo_bit = sdo_val{1,1};
            else
                sdo_bit = sdo_val(1);
            end
            data_out = bitor(data_out, bitshift(double(sdo_bit > 0), i));
        end
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);
    end
    
    ps = set_pin(dq, ps, cs, 1, hw);
end


clear; clc; close all;


USE_HARDWARE = false;


% Pin definitions in a struct (accessible by local functions)
pins.SCLK    = 0;   % P0.0 — output
pins.SDIO    = 1;   % P0.1 — output (data to IC)
pins.CS_TX   = 2;   % P0.2 — output
pins.CS_RX   = 3;   % P0.3 — output
pins.MADV_TX = 4;   % P0.4 — output
pins.MRST_TX = 5;   % P0.5 — output
pins.TxADV   = 6;   % P0.6 — output
pins.TxRST   = 7;   % P0.7 — output
% SDO is read via a separate input session (P1.0)

% Multiplier settings for 16-20 GHz
MULT_EN_16_20   = hex2dec('6E');
MULT_PASS_16_20 = hex2dec('9F');

% Radar parameters
BW = 4e9; T_SWEEP = 1e-3; C = 3e8;
NUM_TX = 4; NUM_RX = 4;
SAMPLE_RATE = 1e6;
NUM_SAMPLES = round(SAMPLE_RATE * T_SWEEP);

% Initial port state (8 output lines)
ps = zeros(1, 8);
ps(pins.CS_TX + 1) = 1;  % CS high = deselected
ps(pins.CS_RX + 1) = 1;

dq = []; dq_in = [];
if USE_HARDWARE
    % Output session: P0.0–P0.7
    dq = daq('ni');
    for i = 0:7
        addoutput(dq, DEV_NAME, sprintf('port0/line%d', i), 'Digital');
    end
    write(dq, ps);
    
    % Input session: P1.0 = SDO (directly from ADAR ICs)
    % Wire: ADAR2001 Pin 15 (SDO) + ADAR2004 Pin 24 (SDO) → P1.0
    % Both SDO lines can share one wire since only one CS is low at a time.
    dq_in = daq('ni');
    addinput(dq_in, DEV_NAME, 'port1/line0', 'Digital');
    
    fprintf('USB-6356 initialized (outputs P0.0-7, input P1.0=SDO).\n\n');
else
    fprintf('SIMULATION MODE\n\n');
end

%% STEP 0: Verify SPI (write → read back → compare)
fprintf('========== STEP 0: SPI Verification ==========\n');
% Write a known value to the scratch-pad register (0x00A) on each IC,
% then read it back and compare. This proves the entire 24-bit SPI path
% works: MATLAB → USB-6356 → level shifter → IC → latch → readback.

test_val_tx = hex2dec('A5');
test_val_rx = hex2dec('5A');

% Write scratch pads
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('00A'), test_val_tx, USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('00A'), test_val_rx, USE_HARDWARE);

% Read back and verify
[ps, rd_tx] = spi_rd(dq, dq_in, ps, pins, pins.CS_TX, hex2dec('00A'), USE_HARDWARE);
[ps, rd_rx] = spi_rd(dq, dq_in, ps, pins, pins.CS_RX, hex2dec('00A'), USE_HARDWARE);

if USE_HARDWARE
    if rd_tx == test_val_tx
        fprintf('  TX SPI PASS: wrote 0x%02X, read 0x%02X ✓\n', test_val_tx, rd_tx);
    else
        fprintf('  TX SPI FAIL: wrote 0x%02X, read 0x%02X ✗\n', test_val_tx, rd_tx);
        error('ADAR2001 SPI verification failed! Check wiring, level shifters, power.');
    end
    if rd_rx == test_val_rx
        fprintf('  RX SPI PASS: wrote 0x%02X, read 0x%02X ✓\n', test_val_rx, rd_rx);
    else
        fprintf('  RX SPI FAIL: wrote 0x%02X, read 0x%02X ✗\n', test_val_rx, rd_rx);
        error('ADAR2004 SPI verification failed! Check wiring, level shifters, power.');
    end
else
    fprintf('  [SIM] Skipping readback (no hardware). Write-only test passed.\n');
end
fprintf('\n');

%% STEP 1: Init ADAR2001 (TX)
fprintf('========== STEP 1: Init ADAR2001 ==========\n');
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('000'), hex2dec('81'), USE_HARDWARE); pause(0.01);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('000'), hex2dec('18'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('010'), hex2dec('01'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('011'), hex2dec('BB'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('012'), hex2dec('0B'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('013'), hex2dec('75'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('014'), hex2dec('B5'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('015'), hex2dec('0C'), USE_HARDWARE);
% Disable sequencers but KEEP latch bypass enabled (bit 4 = 1)
% Datasheet: "when using SPI mode, latching must be bypassed"
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('016'), hex2dec('10'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('018'), hex2dec('10'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('047'), MULT_EN_16_20, USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('048'), MULT_PASS_16_20, USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('045'), hex2dec('C0'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('046'), hex2dec('07'), USE_HARDWARE);
fprintf('TX ready: 16-20 GHz, CH1 active.\n\n');

%% STEP 2: Init ADAR2004 (RX)
%  NOTE: ADAR2004 has DIFFERENT bias registers than ADAR2001!
%  Register names and defaults differ — do NOT copy ADAR2001 values.
%  All values below are from ADAR2004 datasheet Rev.A, Table 9/10.
fprintf('========== STEP 2: Init ADAR2004 ==========\n');
% Soft reset
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('000'), hex2dec('81'), USE_HARDWARE); pause(0.01);
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('000'), hex2dec('18'), USE_HARDWARE);
% Power on
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('010'), hex2dec('01'), USE_HARDWARE);
% ADAR2004 bias registers (DIFFERENT from ADAR2001!)
%   0x011: MULT_MID_BIAS=0x5 [7:4], MULT_LOW_BIAS=0x5 [3:0] → 0x55
%   (ADAR2001 uses 0xBB — higher bias, different multiplier design)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('011'), hex2dec('55'), USE_HARDWARE);
%   0x012: MULT_HIGH_BIAS=0x7 [3:0] → 0x07
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('012'), hex2dec('07'), USE_HARDWARE);
%   0x013: LO_AMP2_BIAS=0x7 [7:4], LO_AMP1_BIAS=0x8 [3:0] → 0x78
%   (ADAR2001 has RF_AMP at 0x013 with 0x75 — different block!)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('013'), hex2dec('78'), USE_HARDWARE);
%   0x014: SPLT2_BIAS=0x7 [7:4], SPLT1_BIAS=0xA [3:0] → 0x7A
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('014'), hex2dec('7A'), USE_HARDWARE);
%   0x015: MIX_BIAS=0x2 [7:4], LNA_BIAS=0xA [3:0] → 0x2A
%   (ADAR2001 has PA_BIAS at 0x015 with 0x0C — no LNA/mixer!)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('015'), hex2dec('2A'), USE_HARDWARE);
%   0x016: IFAMP_BIAS=0xC [7:4], reserved [3:0] → 0xC0
%   (BUG WAS: 0x0C — bias in wrong nibble! Would give zero IF gain)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('016'), hex2dec('C0'), USE_HARDWARE);
%   0x017: IFAMP_CM=0x4 [3:0] → 0x04 (IF output common-mode voltage)
%   (BUG WAS: 0x00 — wrong common-mode, could clip IF output)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('017'), hex2dec('04'), USE_HARDWARE);
% Disable both sequencers (SPI-only control)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('018'), hex2dec('00'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('019'), hex2dec('00'), USE_HARDWARE);
% LO multiplier: mid band active, low+high ready, BPF=low, LO amp on
%   Table 7: 4-5 GHz input → 16-20 GHz LO → value = 0xEE
%   (BUG WAS: 0xED — high band ACT=1 with RDY=0, invalid state)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('02F'), hex2dec('EE'), USE_HARDWARE);
% RX enable: LNA + Mixer + IF amp + all 4 channels ON
%   Bit 7: LNA_EN=1, Bit 6: MIX_EN=1, Bit 5: IFAMP_EN=1
%   Bits 3-0: CH1-CH4 all enabled
%   (BUG WAS: 0x6F — LNA_EN=0, no signal reaches mixers!)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('02B'), hex2dec('EF'), USE_HARDWARE);
% Splitters: all 3 stages enabled
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('02E'), hex2dec('07'), USE_HARDWARE);
% Gain: all channels max (0x7 = max VGA gain)
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('02C'), hex2dec('77'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_RX, hex2dec('02D'), hex2dec('77'), USE_HARDWARE);
fprintf('RX ready: 16-20 GHz LO, 4 ch, max gain.\n\n');

%% STEP 3: MIMO state machines
fprintf('========== STEP 3: MIMO state machines ==========\n');
% Mult modes: 0=sleep, 1=16-20GHz
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('070'), hex2dec('00'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('071'), hex2dec('00'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('072'), MULT_EN_16_20, USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('073'), MULT_PASS_16_20, USE_HARDWARE);
% TX modes: 0=sleep, 1=CH1, 2=CH2, 3=CH3, 4=CH4
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('050'), hex2dec('00'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('051'), hex2dec('00'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('052'), hex2dec('C0'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('053'), hex2dec('07'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('054'), hex2dec('30'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('055'), hex2dec('07'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('056'), hex2dec('0C'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('057'), hex2dec('07'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('058'), hex2dec('03'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('059'), hex2dec('07'), USE_HARDWARE);
% State assignments
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('03C'), hex2dec('10'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('019'), hex2dec('12'), USE_HARDWARE);
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('01A'), hex2dec('34'), USE_HARDWARE);
% Enable sequencers (latch bypass ON for both — faster switching)
% 0x018: MULT_SEQ_EN=1, MULT_CTL_LATCH_BYP=1, depth=0 (1 mult state)
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('018'), hex2dec('90'), USE_HARDWARE);
% 0x016: TX_SEQ_EN=1, TX_CTL_LATCH_BYP=1
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('016'), hex2dec('90'), USE_HARDWARE);
% 0x017: TX_STATES=3 → 4 states in loop (CH1→CH2→CH3→CH4→repeat)
ps = spi_wr(dq, ps, pins, pins.CS_TX, hex2dec('017'), hex2dec('03'), USE_HARDWARE);
fprintf('State machines ready.\n\n');

%% STEP 4: Capture MIMO data
fprintf('========== STEP 4: MIMO capture ==========\n');
if_data = zeros(NUM_SAMPLES, NUM_RX, NUM_TX);

if ~USE_HARDWARE
    R_target = 5.0;
    f_beat = 2 * R_target * BW / (C * T_SWEEP);
    fprintf('[SIM] Target at %.1f m, f_beat = %.1f kHz\n', R_target, f_beat/1e3);
    t = (0:NUM_SAMPLES-1)' / SAMPLE_RATE;
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            ph = 2*pi*(tx-1)*0.3 + 2*pi*(rx-1)*0.1;
            if_data(:, rx, tx) = 0.1*sin(2*pi*f_beat*t + ph) + 0.001*randn(NUM_SAMPLES,1);
        end
    end
    fprintf('[SIM] 16 virtual channels generated.\n\n');
end

%% STEP 5: Range processing
fprintf('========== STEP 5: Range processing ==========\n');
delta_R = C / (2 * BW);
R_max = C * SAMPLE_RATE * T_SWEEP / (4 * BW);
N_FFT = 2^nextpow2(NUM_SAMPLES);
range_ax = linspace(0, R_max, N_FFT/2);
rp = zeros(N_FFT/2, NUM_RX, NUM_TX);

fprintf('Resolution: %.2f cm, Max range: %.1f m\n', delta_R*100, R_max);

figure('Name','Range Profiles','Position',[100 100 1200 800]);
for tx = 1:NUM_TX
    for rx = 1:NUM_RX
        s = if_data(:,rx,tx) .* hanning(NUM_SAMPLES);
        sp = fft(s, N_FFT);
        rp(:,rx,tx) = abs(sp(1:N_FFT/2));
    end
    subplot(2,2,tx);
    avg = mean(rp(:,:,tx), 2);
    plot(range_ax, 20*log10(avg/max(avg)), 'LineWidth', 1.5);
    xlabel('Range (m)'); ylabel('dB');
    title(sprintf('TX%d', tx));
    xlim([0 min(20,R_max)]); ylim([-60 0]); grid on;
end
sgtitle('FMCW Range Profiles');

% MIMO combination
all_ch = zeros(N_FFT/2, NUM_TX*NUM_RX);
for tx = 1:NUM_TX
    for rx = 1:NUM_RX
        all_ch(:,(tx-1)*NUM_RX+rx) = rp(:,rx,tx);
    end
end
coh = sum(all_ch, 2);
[~, pk] = max(coh);
fprintf('Target detected at: %.2f m\n', range_ax(pk));

figure('Name','MIMO Combined');
plot(range_ax, 20*log10(coh/max(coh)), 'LineWidth', 1.5);
xlabel('Range (m)'); ylabel('dB');
title(sprintf('MIMO Combined (16 ch) — Target at %.2f m', range_ax(pk)));
xlim([0 min(20,R_max)]); ylim([-60 0]); grid on;

fprintf('\nDONE. Set USE_HARDWARE=true for real operation.\n');

%% ================================================================
%  LOCAL FUNCTIONS
%  ================================================================

function ps = set_pin(dq, ps, pin, val, hw)
    ps(pin + 1) = val;
    if hw, write(dq, ps); end
end

function ps = spi_wr(dq, ps, pins, cs, addr, data, hw)
    % 24-bit SPI write: [R/W(1)][ADDR(15)][DATA(8)], MSB first
    %   Bit 23   = 0 (write)
    %   Bit 22-8 = A14..A0 (15-bit register address)
    %   Bit 7-0  = D7..D0  (8-bit data)
    word = bitor(bitshift(bitand(addr, hex2dec('7FFF')), 8), ...
                 bitand(data, hex2dec('FF')));
    % R/W = 0 for write (bit 23 already 0)
    ps = set_pin(dq, ps, cs, 0, hw);       % CS low — start transaction
    for i = 23:-1:0                         % 24 clocks, MSB first
        b = bitand(bitshift(word, -i), 1);
        ps = set_pin(dq, ps, pins.SDIO, b, hw);
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);   % rising edge: IC samples
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);   % falling edge: shift next
    end
    ps = set_pin(dq, ps, cs, 1, hw);       % CS high — latch data
    if cs == pins.CS_TX, n='TX'; else, n='RX'; end
    fprintf('  SPI %s: 0x%03X = 0x%02X (word=0x%06X)\n', n, addr, data, word);
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
    %   Bit 7-0  = don't care (IC drives SDO with register contents)
    %
    % 4-wire mode (default): IC outputs data on SDO pin.
    % Hardware wiring: ADAR2001 Pin15 (SDO) + ADAR2004 Pin24 (SDO)
    %                  → USB-6356 P1.0 (digital input)
    % Both SDO lines share one wire — only the selected IC (CS low) drives.
    %
    % Timing: max 15 MHz for read operations (we bit-bang much slower).
    
    % Build instruction word: R/W=1, then 15-bit address
    instr = bitor(bitshift(1, 23), ...            % bit 23 = 1 (read)
                  bitshift(bitand(addr, hex2dec('7FFF')), 8));  % addr in bits 22:8
    
    data_out = 0;
    
    ps = set_pin(dq, ps, cs, 0, hw);       % CS low — start transaction
    
    % --- Instruction phase: clock out 16 bits (R/W + A14..A0) ---
    for i = 23:-1:8
        b = bitand(bitshift(instr, -i), 1);
        ps = set_pin(dq, ps, pins.SDIO, b, hw);
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);   % rising edge
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);   % falling edge
    end
    
    % --- Data phase: clock in 8 bits from SDO ---
    % SDIO = don't care during read; set to 0
    ps = set_pin(dq, ps, pins.SDIO, 0, hw);
    for i = 7:-1:0
        ps = set_pin(dq, ps, pins.SCLK, 1, hw);   % rising edge: SDO valid
        if hw
            sdo_val = read(dq_in);                  % sample SDO (P1.0)
            if istable(sdo_val)
                sdo_bit = sdo_val{1,1};             % extract from table
            else
                sdo_bit = sdo_val(1);
            end
            data_out = bitor(data_out, bitshift(double(sdo_bit > 0), i));
        end
        ps = set_pin(dq, ps, pins.SCLK, 0, hw);   % falling edge
    end
    
    ps = set_pin(dq, ps, cs, 1, hw);       % CS high — end transaction
    
    if cs == pins.CS_TX, n='TX'; else, n='RX'; end
    if hw
        fprintf('  SPI %s READ: 0x%03X = 0x%02X\n', n, addr, data_out);
    else
        fprintf('  SPI %s READ: 0x%03X [sim — no readback]\n', n, addr);
    end
end
% plot_transmitter_csvs.m
% --------------------------------------------------------------
% Reads spectrum-analyzer CSV files and generates plots of the
% FMCW MIMO transmitter measurements.
%
% Place ALL CSVs (CW, isolation, sweeps, spurs) in a single folder.
% Edit CSV_DIR and OUT_DIR below, then just run the script.
%
% Expected files in CSV_DIR (script skips any that are missing):
%   1.csv, 2.csv, 3.csv, 4.csv                  : per-channel CW @ 18 GHz
%   1 Isolation at 2.csv                        : TX1 active, measure TX2
%   1 Isolation at 3.csv                        : TX1 active, measure TX3
%   1 Isolation at 4.csv                        : TX1 active, measure TX4
%   2 Isolation 1.csv                           : TX2 active, measure TX1
%   2 Isolation 3.csv                           : TX2 active, measure TX3
%   2 Isolation at 4.csv                        : TX2 active, measure TX4
%   Sweep 16-20GHz 1st Channel- Max_Hold.csv    : 16-20 GHz max-hold per channel
%   Sweep 16-20GHz 2nd Channel - Max hold.csv
%   Sweep 16-20GHz 3rd Channel - Max hold.csv
%   Sweep 16-20GHz 4th Channel - Max hold.csv
%   Spurs_Testing.csv                           : narrow-span spurs around 18 GHz
%
% Each CSV row: "freq_Hz, power_dBm"  (no header, comma-space separator)
%
% Produces 4 PNG figures in OUT_DIR:
%   1. tx_cw_per_channel.png
%   2. tx_isolation.png
%   3. tx_sweep_16_20GHz.png
%   4. tx_spurs.png
%
% Compatible with MATLAB R2016b+ (local functions in scripts) and Octave.
% --------------------------------------------------------------

clear; close all; clc;

% =========== CONFIGURATION — edit these two paths if needed ===========
CSV_DIR = '.';            % directory containing all CSVs
OUT_DIR = './Images';     % output directory for PNG plots
% ======================================================================

if ~exist(OUT_DIR, 'dir'), mkdir(OUT_DIR); end

fprintf('Reading CSVs from: %s\n', CSV_DIR);
fprintf('Saving plots to:    %s\n\n', OUT_DIR);

fp = @(name) fullfile(CSV_DIR, name);

% =====================================================================
% FIGURE 1: Per-channel CW at 18 GHz (2x2 subplot)
% =====================================================================
fprintf('Plotting per-channel CW measurements...\n');
fig1 = figure('Name', 'TX CW per channel', 'Position', [100 100 1100 800], 'Color', 'w');
channels = {'1.csv', '2.csv', '3.csv', '4.csv'};
for ch = 1:4
    D = read_csv_pair(fp(channels{ch}));
    if isempty(D), continue; end
    [~, idx18] = min(abs(D(:,1) - 18.0e9));
    pwr_at_18 = D(idx18, 2);

    subplot(2, 2, ch);
    plot(D(:,1)/1e9, D(:,2), 'b-', 'LineWidth', 0.5);
    grid on; box on;
    xlabel('Frequency (GHz)', 'FontSize', 11);
    ylabel('Power (dBm)',     'FontSize', 11);
    title(sprintf('TX Channel %d: %.2f dBm @ 18 GHz', ch, pwr_at_18), 'FontSize', 12);
    xlim([min(D(:,1))/1e9, max(D(:,1))/1e9]);
    ylim([-120, 5]);
end
super_title('Transmitter CW Measurements at 18 GHz (per channel)');
save_fig(fig1, fullfile(OUT_DIR, 'tx_cw_per_channel.png'));

% =====================================================================
% FIGURE 2: Isolation / cross-channel leakage measurements
%   "X Isolation Y" or "X Isolation at Y" = TX X is active, port Y measured
% =====================================================================
fprintf('Plotting isolation measurements...\n');

isolation_list = {
    '1 Isolation at 2.csv', 'TX1 active, leakage at TX2 port';
    '1 Isolation at 3.csv', 'TX1 active, leakage at TX3 port';
    '1 Isolation at 4.csv', 'TX1 active, leakage at TX4 port';
    '2 Isolation 1.csv',    'TX2 active, leakage at TX1 port';
    '2 Isolation 3.csv',    'TX2 active, leakage at TX3 port';
    '2 Isolation at 4.csv', 'TX2 active, leakage at TX4 port';
};

avail_idx = [];
for k = 1:size(isolation_list, 1)
    if exist(fp(isolation_list{k, 1}), 'file')
        avail_idx(end+1) = k;
    end
end

if ~isempty(avail_idx)
    fig2 = figure('Name', 'TX Isolation', 'Position', [100 100 1100 800], 'Color', 'w');
    n_avail = length(avail_idx);
    nrows = ceil(n_avail / 2);
    for kk = 1:n_avail
        k = avail_idx(kk);
        D = read_csv_pair(fp(isolation_list{k, 1}));
        if isempty(D), continue; end
        [~, idx18] = min(abs(D(:,1) - 18.0e9));
        iso_dBm = D(idx18, 2);

        subplot(nrows, 2, kk);
        plot(D(:,1)/1e9, D(:,2), 'r-', 'LineWidth', 0.5);
        grid on; box on;
        xlabel('Frequency (GHz)', 'FontSize', 10);
        ylabel('Power (dBm)',     'FontSize', 10);
        title(sprintf('%s: %.1f dBm @ 18 GHz', isolation_list{k, 2}, iso_dBm), 'FontSize', 11);
        xlim([min(D(:,1))/1e9, max(D(:,1))/1e9]);
        ylim([-125, -30]);
    end
    super_title('Cross-Channel Isolation (Carrier Leakage at 18 GHz)');
    save_fig(fig2, fullfile(OUT_DIR, 'tx_isolation.png'));
else
    fprintf('  No isolation CSV files found, skipping isolation plot.\n');
end

% =====================================================================
% FIGURE 3: 16-20 GHz max-hold sweeps (2x2 subplot)
%   Raw max-hold trace as captured by the spectrum analyzer.
%   Note: bin-to-bin gaps are real measurement artifacts from the
%   non-uniform VCO Vtune slope during the chirp sweep.
% =====================================================================
fprintf('Plotting 16-20 GHz max-hold sweeps...\n');

sweep_names = {
    {'Sweep 16-20GHz 1st Channel- Max_Hold.csv', ...
     'Sweep 16-20GHz 1st Channel - Max_Hold.csv'};
    {'Sweep 16-20GHz 2nd Channel - Max hold.csv', ...
     'Sweep 16-20GHz 2nd Channel- Max_Hold.csv'};
    {'Sweep 16-20GHz 3rd Channel - Max hold.csv', ...
     'Sweep 16-20GHz 3rd Channel- Max_Hold.csv'};
    {'Sweep 16-20GHz 4th Channel - Max hold.csv', ...
     'Sweep 16-20GHz 4th Channel- Max_Hold.csv'};
};

fig3 = figure('Name', 'TX Sweep 16-20 GHz', 'Position', [100 100 1100 800], 'Color', 'w');
found_any_sweep = false;
for ch = 1:4
    D = [];
    for cand = 1:length(sweep_names{ch})
        cp = fp(sweep_names{ch}{cand});
        if exist(cp, 'file'), D = read_csv_pair(cp); break; end
    end
    if isempty(D)
        fprintf('  Warning: no sweep file found for channel %d\n', ch);
        continue;
    end
    found_any_sweep = true;
    [peak_dBm, peak_idx] = max(D(:,2));
    peak_GHz = D(peak_idx, 1) / 1e9;

    subplot(2, 2, ch);
    plot(D(:,1)/1e9, D(:,2), 'Color', [0.8 0.2 0.2], 'LineWidth', 0.5);
    grid on; box on;
    xlabel('Frequency (GHz)', 'FontSize', 11);
    ylabel('Power (dBm)',     'FontSize', 11);
    title(sprintf('Channel %d Max-Hold: peak %.2f dBm @ %.2f GHz', ch, peak_dBm, peak_GHz), 'FontSize', 12);
    xlim([15.9, 20.1]);
    ylim([-65, 5]);
end
if found_any_sweep
    super_title('16-20 GHz Max-Hold Sweeps');
    save_fig(fig3, fullfile(OUT_DIR, 'tx_sweep_16_20GHz.png'));
else
    close(fig3);
end

% =====================================================================
% FIGURE 4: Spurs around 18 GHz carrier
%   Detects spurs (local maxima above noise floor, excluding carrier
%   skirt) and annotates each with frequency offset and power in dBc.
% =====================================================================
fprintf('Plotting spurs measurement...\n');
if exist(fp('Spurs_Testing.csv'), 'file')
    D = read_csv_pair(fp('Spurs_Testing.csv'));
    freqs = D(:,1);
    pwrs  = D(:,2);
    [carrier_dBm, carrier_idx] = max(pwrs);
    carrier_GHz = freqs(carrier_idx) / 1e9;

    % Estimate noise floor as 10th percentile of all bins
    sorted_p = sort(pwrs);
    noise_floor = sorted_p(round(0.10 * length(sorted_p)));

    % Detect spurs: local maxima >= noise_floor + 8 dB, excluding
    % carrier skirt within ±500 kHz of the carrier
    spur_threshold = noise_floor + 8;
    skirt_Hz = 500e3;
    win_bins = 50;   % declare a peak if it's max in ±50 bins
    N = length(pwrs);
    spur_list = [];   % rows: [freq_GHz, power_dBm, offset_MHz, dBc]
    for i = (win_bins+1) : (N - win_bins)
        if pwrs(i) < spur_threshold, continue; end
        if abs(freqs(i) - freqs(carrier_idx)) < skirt_Hz, continue; end
        lo = i - win_bins;
        hi = i + win_bins;
        if pwrs(i) >= max(pwrs(lo:hi))
            offset_MHz = (freqs(i) - freqs(carrier_idx)) / 1e6;
            dBc = pwrs(i) - carrier_dBm;
            spur_list(end+1, :) = [freqs(i)/1e9, pwrs(i), offset_MHz, dBc];
        end
    end

    fig4 = figure('Name', 'TX Spurs', 'Position', [100 100 1200 600], 'Color', 'w');
    plot(freqs/1e9, pwrs, 'k-', 'LineWidth', 0.7);
    grid on; box on;
    hold on;

    % Annotate carrier (stack labels above the peak)
    plot(carrier_GHz, carrier_dBm, 'b^', 'MarkerSize', 10, ...
         'MarkerFaceColor', 'b', 'LineWidth', 1.0);
    text(carrier_GHz, carrier_dBm + 5, ...
         sprintf('Carrier %.4f GHz', carrier_GHz), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, ...
         'FontWeight', 'bold', 'Color', 'b');
    text(carrier_GHz, carrier_dBm + 2.5, ...
         sprintf('%.2f dBm', carrier_dBm), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, ...
         'FontWeight', 'bold', 'Color', 'b');

    % Annotate each spur with stacked single-line labels
    for s = 1:size(spur_list, 1)
        spur_GHz   = spur_list(s, 1);
        spur_dBm   = spur_list(s, 2);
        spur_off   = spur_list(s, 3);
        spur_dBc   = spur_list(s, 4);

        plot(spur_GHz, spur_dBm, 'rv', 'MarkerSize', 9, ...
             'MarkerFaceColor', 'r', 'LineWidth', 1.0);
        % Stack three single-line labels above the marker
        text(spur_GHz, spur_dBm + 10, ...
             sprintf('%.3f GHz', spur_GHz), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', 'r');
        text(spur_GHz, spur_dBm + 7, ...
             sprintf('%+.2f MHz', spur_off), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', 'r');
        text(spur_GHz, spur_dBm + 4, ...
             sprintf('%.1f dBm (%.1f dBc)', spur_dBm, spur_dBc), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', 'r');
    end

    hold off;
    xlabel('Frequency (GHz)', 'FontSize', 12);
    ylabel('Power (dBm)',     'FontSize', 12);
    title(sprintf('TX Spurs around 18 GHz Carrier (%d spurs detected, threshold = noise+8 dB)', ...
                  size(spur_list, 1)), 'FontSize', 13);
    xlim([min(freqs)/1e9, max(freqs)/1e9]);
    ylim([-85, 15]);
    save_fig(fig4, fullfile(OUT_DIR, 'tx_spurs.png'));

    % Print spur summary to console
    fprintf('  Carrier:   %.4f GHz at %.2f dBm\n', carrier_GHz, carrier_dBm);
    fprintf('  Noise floor estimate: %.1f dBm\n', noise_floor);
    fprintf('  Detected spurs:\n');
    for s = 1:size(spur_list, 1)
        fprintf('    %.4f GHz (%+.2f MHz offset): %.2f dBm (%.1f dBc)\n', ...
                spur_list(s, 1), spur_list(s, 3), spur_list(s, 2), spur_list(s, 4));
    end
else
    fprintf('  Spurs_Testing.csv not found, skipping spurs plot.\n');
end

fprintf('\nAll plots saved to: %s\n', OUT_DIR);

% =====================================================================
%                       LOCAL HELPER FUNCTIONS
% (MATLAB R2016b+ supports local functions in scripts. Octave supports it too.)
% =====================================================================

function D = read_csv_pair(filename)
    % Read "freq_Hz, power_dBm" CSV; returns Nx2 matrix or [] if missing.
    if ~exist(filename, 'file')
        warning('File not found: %s', filename);
        D = [];
        return;
    end
    try
        D = dlmread(filename, ',', 0, 0);
    catch
        fid = fopen(filename, 'r');
        C = textscan(fid, '%f%f', 'Delimiter', ',', 'CollectOutput', true);
        fclose(fid);
        D = C{1};
    end
end

function y = moving_max(x, win)
    % Symmetric moving maximum, window size win (odd).
    if win < 2, y = x; return; end
    if mod(win, 2) == 0, win = win + 1; end
    half = (win - 1) / 2;
    N = length(x);
    y = zeros(size(x));
    for i = 1:N
        lo = max(1, i - half);
        hi = min(N, i + half);
        y(i) = max(x(lo:hi));
    end
end

function super_title(txt)
    try
        sgtitle(txt, 'FontSize', 13, 'FontWeight', 'bold');
    catch
        try, suptitle(txt); catch
            annotation('textbox', [0 0.95 1 0.04], 'String', txt, ...
                       'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
                       'FontSize', 13, 'FontWeight', 'bold');
        end
    end
end

function save_fig(figH, filename)
    try
        saveas(figH, filename);
        fprintf('  Saved: %s\n', filename);
    catch ME
        fprintf('  Failed to save %s: %s\n', filename, ME.message);
    end
end
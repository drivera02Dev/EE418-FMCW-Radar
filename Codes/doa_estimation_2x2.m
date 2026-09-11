

%% DOA Estimation — 2 TX × 2 RX configuration (sparse virtual array)
%
% Active channels: TX1+TX4, RX1+RX4
% Virtual array: 4 sparse elements at [0, 1.5λ, 6λ, 7.5λ]
%
% Note: sparse arrays have grating-lobe ambiguity. Beamforming and MUSIC
% will show multiple peaks for a single source — true DOA is among them.
% Use a priori range/multipath info to disambiguate, or repair antennas
% to return to the well-sampled 4×4 array.

clear; clc; close all;

c       = 3e8;
fstart  = 16e9;
fstop   = 20e9;
BW      = fstop - fstart;
fc      = (fstart + fstop) / 2;
lambda  = c / fc;
Tp      = 1e-3;
FS      = 1e6;
N       = round(Tp * FS);

% Active channels — change to 1:4 once damaged antennas repaired
TX_CHANNELS = [1 4];
RX_CHANNELS = [1 4];
NUM_TX = length(TX_CHANNELS);
NUM_RX = length(RX_CHANNELS);
NUM_VIRT = NUM_TX * NUM_RX;

d_tx = 2 * lambda;
d_rx = lambda / 2;

% Virtual element positions (NOT uniform — 4 sparse positions)
virt_pos = zeros(NUM_VIRT, 1);
vi = 0;
for ti = 1:NUM_TX
    for ri = 1:NUM_RX
        vi = vi + 1;
        virt_pos(vi) = (TX_CHANNELS(ti) - 1) * d_tx + (RX_CHANNELS(ri) - 1) * d_rx;
    end
end

rr = c / (2 * BW);
max_range = rr * N / 2;
zpad = 8 * N / 2;
range_axis = linspace(0, max_range, zpad/2);
dbv = @(x) 20*log10(abs(x) + eps);

% Number of frames for covariance averaging
NUM_FRAMES = 50;

fprintf('============================================================\n');
fprintf('FMCW MIMO RADAR — DOA Estimation (Multi-snapshot)\n');
fprintf('============================================================\n');
fprintf('Virtual array: %d sparse elements at [%s]λ\n', NUM_VIRT, ...
    sprintf('%.1f ', virt_pos / lambda));
fprintf('Snapshots for covariance: %d frames\n', NUM_FRAMES);
fprintf('\n');

%% Targets: [range_m, amplitude, angle_deg]
targets = [
    5.0,  0.10,  -20;
    5.0,  0.08,   15;
    8.0,  0.06,    0;
   12.0,  0.04,   30;
];

fprintf('Targets:\n');
for i = 1:size(targets,1)
    fprintf('  R=%.1f m, θ=%+.0f°\n', targets(i,1), targets(i,3));
end
fprintf('\n');

%% Generate multi-frame data: [N x NUM_RX x NUM_TX x NUM_FRAMES]
fprintf('Generating %d frames of data...\n', NUM_FRAMES);
sif_all = zeros(N, NUM_RX, NUM_TX, NUM_FRAMES);
t_sample = (0:N-1)' / FS;

for frame = 1:NUM_FRAMES
    % Independent random phase for EACH target in this frame
    % Same phase across all TX/RX (coherent within one frame)
    % Different across frames (target fluctuation / slight motion)
    phase_rand = 2 * pi * rand(size(targets, 1), 1);
    
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            sig = zeros(N, 1);
            for tgt = 1:size(targets, 1)
                R     = targets(tgt, 1);
                amp   = targets(tgt, 2);
                theta = targets(tgt, 3) * pi / 180;
                
                f_beat = 2 * R * BW / (c * Tp);
                phase_rt = 4 * pi * R * fc / c;
                pos = (TX_CHANNELS(tx)-1)*d_tx + (RX_CHANNELS(rx)-1)*d_rx;
                phase_arr = 2 * pi * pos * sin(theta) / lambda;
                
                sig = sig + amp * exp(1j*(2*pi*f_beat*t_sample ...
                    + phase_rt + phase_arr + phase_rand(tgt)));
            end
            sig = sig + 0.003*(randn(N,1) + 1j*randn(N,1));
            sif_all(:, rx, tx, frame) = sig;
        end
    end
end

%% Range FFT — all frames, all channels (keep complex)
fprintf('Computing range FFTs...\n');
% [range_bins x virtual_channels x frames]
range_profiles = zeros(zpad/2, NUM_VIRT, NUM_FRAMES);

for frame = 1:NUM_FRAMES
    vi = 0;
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            vi = vi + 1;
            sig = sif_all(:, rx, tx, frame);
            spec = fft(sig .* hanning(N), zpad);
            range_profiles(:, vi, frame) = spec(1:zpad/2);
        end
    end
end

%% Detect target range bins (use magnitude sum across all channels+frames)
mag_sum = sum(sum(abs(range_profiles), 2), 3);
[~, peak_bins] = findpeaks(dbv(mag_sum), ...
    'MinPeakHeight', max(dbv(mag_sum)) - 15, ...
    'MinPeakDistance', round(0.3 / (range_axis(2)-range_axis(1))));

fprintf('Detected %d range bins:\n', length(peak_bins));
for i = 1:length(peak_bins)
    fprintf('  %.2f m\n', range_axis(peak_bins(i)));
end

%% Steering vector
steer = @(th) exp(1j*2*pi*virt_pos*sin(th)/lambda);   % sparse-array steering

%% DOA at each range bin — with multi-snapshot covariance
theta_scan = -90:0.5:90;
N_theta = length(theta_scan);

for pk = 1:length(peak_bins)
    bin = peak_bins(pk);
    R_det = range_axis(bin);
    
    % ---- Build covariance from ALL frames ----
    Rxx = zeros(NUM_VIRT);
    for frame = 1:NUM_FRAMES
        x = range_profiles(bin, :, frame).';  % [16x1]
        Rxx = Rxx + (x * x');
    end
    Rxx = Rxx / NUM_FRAMES;  % Average over snapshots
    
    % Diagonal loading (very small — just for numerical stability)
    Rxx = Rxx + 1e-4 * eye(NUM_VIRT) * trace(Rxx) / NUM_VIRT;
    
    % ---- FFT Beamforming ----
    P_fft = zeros(N_theta, 1);
    for i = 1:N_theta
        a = steer(theta_scan(i)*pi/180);
        P_fft(i) = abs(a' * Rxx * a);
    end
    P_fft_dB = 10*log10(P_fft/max(P_fft));
    
    % ---- Capon (MVDR) ----
    Rxx_inv = inv(Rxx);
    P_capon = zeros(N_theta, 1);
    for i = 1:N_theta
        a = steer(theta_scan(i)*pi/180);
        P_capon(i) = 1 / real(a' * Rxx_inv * a);
    end
    P_capon_dB = 10*log10(P_capon/max(P_capon));
    
    % ---- MUSIC ----
    [V, D] = eig(Rxx);
    [eigvals, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    
    % Estimate number of sources using eigenvalue ratio test
    % Look for the biggest drop between consecutive eigenvalues
    eig_ratios = eigvals(1:end-1) ./ eigvals(2:end);
    [~, max_drop] = max(eig_ratios);
    num_src = max_drop;  % sources = index of biggest ratio jump
    num_src = max(num_src, 1);
    num_src = min(num_src, 4);
    
    % Noise subspace
    En = V(:, num_src+1:end);
    
    P_music = zeros(N_theta, 1);
    for i = 1:N_theta
        a = steer(theta_scan(i)*pi/180);
        P_music(i) = 1 / real(a' * (En * En') * a);
    end
    P_music_dB = 10*log10(P_music/max(P_music));
    
    % ---- Plot ----
    figure('Name', sprintf('DOA R=%.1fm', R_det), ...
        'Position', [50+pk*60 50+pk*60 900 550]);
    
    subplot(3,1,1);
    plot(theta_scan, P_fft_dB, 'b', 'LineWidth', 1.5);
    ylabel('dB'); title(sprintf('FFT beamforming — R = %.2f m', R_det));
    xlim([-90 90]); ylim([-40 0]); grid on;
    hold on;
    for t = 1:size(targets,1)
        if abs(targets(t,1)-R_det) < 0.5
            xline(targets(t,3), '--r', sprintf('%+.0f°',targets(t,3)), 'LineWidth', 1.2);
        end
    end; hold off;
    
    subplot(3,1,2);
    plot(theta_scan, P_capon_dB, 'Color', [0.85 0.33 0.1], 'LineWidth', 1.5);
    ylabel('dB'); title(sprintf('Capon (MVDR) — %d snapshots', NUM_FRAMES));
    xlim([-90 90]); ylim([-40 0]); grid on;
    hold on;
    for t = 1:size(targets,1)
        if abs(targets(t,1)-R_det) < 0.5
            xline(targets(t,3), '--r', sprintf('%+.0f°',targets(t,3)), 'LineWidth', 1.2);
        end
    end; hold off;
    
    subplot(3,1,3);
    plot(theta_scan, P_music_dB, 'Color', [0.47 0.29 0.72], 'LineWidth', 1.5);
    xlabel('Angle (degrees)'); ylabel('dB');
    title(sprintf('MUSIC — %d sources estimated', num_src));
    xlim([-90 90]); ylim([-40 0]); grid on;
    hold on;
    for t = 1:size(targets,1)
        if abs(targets(t,1)-R_det) < 0.5
            xline(targets(t,3), '--r', sprintf('%+.0f°',targets(t,3)), 'LineWidth', 1.2);
        end
    end; hold off;
    
    % Print detected angles (from FFT beamforming — most reliable)
    [apks, alocs] = findpeaks(P_fft_dB, ...
        'MinPeakHeight', -6, 'MinPeakDistance', 15);
    fprintf('\n  R = %.2f m (%d sources):\n', R_det, num_src);
    for ai = 1:length(alocs)
        fprintf('    θ = %+.1f° (%.1f dB)\n', theta_scan(alocs(ai)), apks(ai));
    end
end

%% Range-Angle Maps
fprintf('\nComputing Range-Angle maps...\n');

theta_map = -60:0.5:60;
N_ang = length(theta_map);

% --- FFT Range-Angle map ---
N_afft = 128;
ra_fft = zeros(zpad/2, N_afft);
% Use average of all frames for cleaner map
rp_avg = mean(range_profiles, 3);  % [range_bins x 16]
for rbin = 1:zpad/2
    x_snap = rp_avg(rbin, :);
    ra_fft(rbin, :) = abs(fftshift(fft(x_snap, N_afft)));
end
sin_ax = linspace(-1, 1, N_afft);
theta_fft_ax = asind(sin_ax);

% --- Capon Range-Angle map (multi-snapshot) ---
ra_capon = zeros(zpad/2, N_ang);
mag_thresh = max(dbv(mag_sum)) - 25;
active_bins = find(dbv(mag_sum) > mag_thresh);

fprintf('Capon map: processing %d active range bins...\n', length(active_bins));
for ri = 1:length(active_bins)
    rbin = active_bins(ri);
    
    % Multi-snapshot covariance at this range bin
    Rxx_local = zeros(NUM_VIRT);
    for frame = 1:NUM_FRAMES
        x = range_profiles(rbin, :, frame).';
        Rxx_local = Rxx_local + (x * x');
    end
    Rxx_local = Rxx_local / NUM_FRAMES;
    Rxx_local = Rxx_local + 1e-4*eye(NUM_VIRT)*trace(Rxx_local)/NUM_VIRT;
    Rxx_local_inv = inv(Rxx_local);
    
    for ai = 1:N_ang
        a = steer(theta_map(ai)*pi/180);
        ra_capon(rbin, ai) = 1 / real(a' * Rxx_local_inv * a);
    end
end

% --- MUSIC Range-Angle map ---
ra_music = zeros(zpad/2, N_ang);
fprintf('MUSIC map: processing %d active range bins...\n', length(active_bins));
for ri = 1:length(active_bins)
    rbin = active_bins(ri);
    
    Rxx_local = zeros(NUM_VIRT);
    for frame = 1:NUM_FRAMES
        x = range_profiles(rbin, :, frame).';
        Rxx_local = Rxx_local + (x * x');
    end
    Rxx_local = Rxx_local / NUM_FRAMES;
    Rxx_local = Rxx_local + 1e-4*eye(NUM_VIRT)*trace(Rxx_local)/NUM_VIRT;
    
    [V, D] = eig(Rxx_local);
    [ev, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    eig_rat = ev(1:end-1) ./ ev(2:end);
    [~, md] = max(eig_rat);
    ns = max(min(md, 4), 1);
    En = V(:, ns+1:end);
    
    for ai = 1:N_ang
        a = steer(theta_map(ai)*pi/180);
        ra_music(rbin, ai) = 1 / real(a' * (En*En') * a);
    end
end

% --- Plot all three maps ---
figure('Name', 'Range-Angle Maps', 'Position', [100 50 1000 800]);

% FFT map (already has natural width — plot directly)
subplot(3,1,1);
ra_fft_dB = dbv(ra_fft);
imagesc(theta_fft_ax, range_axis, ra_fft_dB - max(ra_fft_dB(:)), [-40 0]);
colormap('jet'); colorbar;
xlabel('Angle (°)'); ylabel('Range (m)');
title('Range-Angle map — FFT beamforming');
xlim([-60 60]); ylim([0 min(18,max_range)]);
set(gca,'YDir','normal');
hold on;
for t = 1:size(targets,1)
    plot(targets(t,3), targets(t,1), 'wo', 'MarkerSize', 14, 'LineWidth', 2);
    plot(targets(t,3), targets(t,1), 'w+', 'MarkerSize', 12, 'LineWidth', 1.5);
end; hold off;

% Capon map — heavy smoothing + tight dynamic range
subplot(3,1,2);
% Large Gaussian kernel to make single-pixel peaks visible
kern_size_r = 30;  % ±30 range bins
kern_size_a = 8;   % ±8 angle bins
[ka, kr] = meshgrid(-kern_size_a:kern_size_a, -kern_size_r:kern_size_r);
gauss_kern = exp(-(kr.^2/(2*12^2) + ka.^2/(2*3^2)));
gauss_kern = gauss_kern / sum(gauss_kern(:));
ra_capon_smooth = conv2(ra_capon, gauss_kern, 'same');
ra_capon_s_dB = dbv(ra_capon_smooth);
m_cap = max(ra_capon_s_dB(:));
imagesc(theta_map, range_axis, ra_capon_s_dB - m_cap, [-20 0]);
colormap('jet'); colorbar;
xlabel('Angle (°)'); ylabel('Range (m)');
title(sprintf('Range-Angle map — Capon (MVDR), %d snapshots', NUM_FRAMES));
xlim([-60 60]); ylim([0 min(18,max_range)]);
set(gca,'YDir','normal');
hold on;
for t = 1:size(targets,1)
    plot(targets(t,3), targets(t,1), 'wo', 'MarkerSize', 14, 'LineWidth', 2);
    plot(targets(t,3), targets(t,1), 'w+', 'MarkerSize', 12, 'LineWidth', 1.5);
end; hold off;

% MUSIC map — same heavy smoothing
subplot(3,1,3);
ra_music_smooth = conv2(ra_music, gauss_kern, 'same');
ra_music_s_dB = dbv(ra_music_smooth);
m_mus = max(ra_music_s_dB(:));
imagesc(theta_map, range_axis, ra_music_s_dB - m_mus, [-20 0]);
colormap('jet'); colorbar;
xlabel('Angle (°)'); ylabel('Range (m)');
title(sprintf('Range-Angle map — MUSIC, %d snapshots', NUM_FRAMES));
xlim([-60 60]); ylim([0 min(18,max_range)]);
set(gca,'YDir','normal');
hold on;
for t = 1:size(targets,1)
    plot(targets(t,3), targets(t,1), 'wo', 'MarkerSize', 14, 'LineWidth', 2);
    plot(targets(t,3), targets(t,1), 'w+', 'MarkerSize', 12, 'LineWidth', 1.5);
end; hold off;

fprintf('\n============================================================\n');
fprintf('DONE\n');
fprintf('============================================================\n');
fprintf('  DOA spectra: 3 figures (one per detected range)\n');
fprintf('  Range-Angle maps: FFT vs Capon vs MUSIC\n');
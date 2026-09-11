

%% MIMO-SAR Imaging — 2 TX × 2 RX configuration (sparse virtual array)
%
% Active channels: TX1+TX4, RX1+RX4 (only working antennas right now)
% Virtual array: 4 sparse elements at [0, 1.5λ, 6λ, 7.5λ]
%
% Note: this sparse spacing produces grating lobes and reduced cross-range
% quality vs the full 4×4 array (16 uniformly-spaced virtual elements).
% Once damaged channels are repaired, set TX_CHANNELS = RX_CHANNELS = 1:4.

clear; clc; close all;

dbv = @(x) 20*log10(abs(x) + eps);

% Radar Parameters
c      = 3e8;
fstart = 16e9;
fstop  = 24e9;
fc     = (fstart + fstop) / 2;  % 20 GHz
BW     = fstop - fstart;         % 8 GHz
lambda = c / fc;                  % 15 mm
Tp     = 1e-3;
FS     = 1e6;
N      = round(Tp * FS);
cr     = BW / Tp;

% Active channels — change to 1:4 once damaged antennas are repaired
TX_CHANNELS = [1 4];
RX_CHANNELS = [1 4];
NUM_TX = length(TX_CHANNELS);
NUM_RX = length(RX_CHANNELS);
NUM_VIRT = NUM_TX * NUM_RX;  % 4

% Array geometry (element-to-element spacing)
d_tx = 2 * lambda;    % nominal TX spacing
d_rx = lambda / 2;    % nominal RX spacing

% Virtual element offsets — sparse positions from active TX/RX channels
virt_offsets = zeros(NUM_VIRT, 1);
vi = 0;
for ti = 1:NUM_TX
    for ri = 1:NUM_RX
        vi = vi + 1;
        % Physical position = (channel index − 1) × element spacing
        virt_offsets(vi) = (TX_CHANNELS(ti) - 1) * d_tx + (RX_CHANNELS(ri) - 1) * d_rx;
    end
end
virt_span = max(virt_offsets) - min(virt_offsets);  % 112.5 mm (same as 4×4)

fprintf('Virtual array: %d sparse elements\n', NUM_VIRT);
fprintf('Element positions: ');
for k = 1:NUM_VIRT, fprintf('%.1f ', virt_offsets(k)*1e3); end
fprintf('mm\n');
fprintf('Virtual span:  %.1f mm\n', virt_span*1e3);

% Rail step for sparse array: use λ/2 increments to fill the gaps between
% stops. With 4 sparse elements per stop, we need many stops to densely
% cover the aperture. Total samples per rail length is reduced 4× vs 4×4.
rail_step   = d_rx;  % step by λ/2 — sweeps fill in along-track positions
NUM_STOPS   = 80;                 % more stops needed for sparse 2×2 (4× vs 4×4)
rail_length = rail_step * (NUM_STOPS - 1);

% Scene center
Rs = 3.0;  % m

% Total synthetic aperture
total_aperture = rail_length + virt_span;
total_elements = NUM_STOPS * NUM_VIRT;

fprintf('Rail step:     %.1f mm\n', rail_step*1e3);
fprintf('Rail stops:    %d\n', NUM_STOPS);
fprintf('Rail length:   %.1f cm\n', rail_length*100);
fprintf('Total aperture: %.1f cm (%d virtual elements)\n', total_aperture*100, total_elements);
fprintf('\n');

% Resolution
range_res = c / (2 * BW);
cross_res = lambda * Rs / (2 * total_aperture);
fprintf('Range resolution:      %.2f cm\n', range_res*100);
fprintf('Cross-range resolution: %.2f cm (at %.1f m)\n', cross_res*100, Rs);

% Compare to single-antenna SAR with same number of stops
single_aperture = d_rx * (NUM_STOPS - 1);  % only 7.5mm steps, 20 stops
single_cross_res = lambda * Rs / (2 * single_aperture);
fprintf('\nComparison — same %d stops, single antenna:\n', NUM_STOPS);
fprintf('  Aperture: %.1f cm → cross-range: %.2f cm\n', single_aperture*100, single_cross_res*100);
fprintf('  MIMO gives %.1fx longer aperture, %.1fx better resolution!\n', ...
    total_aperture/single_aperture, single_cross_res/cross_res);
fprintf('\n');

%% Build all virtual element positions across the full aperture
% Each rail stop p contributes 16 virtual positions:
%   x_virtual(p,v) = rail_pos(p) + virt_offsets(v)

rail_positions = (0:NUM_STOPS-1) * rail_step;  % rail center positions
rail_positions = rail_positions - mean(rail_positions);  % center at 0

all_x_pos = zeros(total_elements, 1);  % all virtual element x positions
idx = 0;
for p = 1:NUM_STOPS
    for v = 1:NUM_VIRT
        idx = idx + 1;
        all_x_pos(idx) = rail_positions(p) + virt_offsets(v);
    end
end
all_x_pos = all_x_pos - mean(all_x_pos);  % center

% Sort by position (important for proper FFT processing)
[all_x_pos_sorted, sort_idx] = sort(all_x_pos);

% Effective uniform spacing
dx_eff = d_rx;  % λ/2 spacing throughout

fprintf('Virtual aperture: %.1f cm to %.1f cm\n', ...
    min(all_x_pos_sorted)*100, max(all_x_pos_sorted)*100);

%% Scene Definition
scene_targets = [
    -0.30,  2.5,  1.0;
     0.00,  3.0,  0.8;
     0.20,  3.0,  0.6;    % 20cm from previous — test cross-range resolution
     0.50,  3.5,  0.4;
    -0.10,  4.0,  0.3;
     0.40,  2.8,  0.5;    % extra target
    -0.50,  3.2,  0.7;    % extra target far left
];

fprintf('\nScene: %d targets\n', size(scene_targets,1));
for i = 1:size(scene_targets,1)
    fprintf('  x=%+.2f m, y=%.1f m\n', scene_targets(i,1), scene_targets(i,2));
end
fprintf('\n');

%% Simulate Data Capture
fprintf('Simulating MIMO-SAR capture (%d stops × %d channels = %d profiles)...\n', ...
    NUM_STOPS, NUM_VIRT, total_elements);

t_sample = linspace(0, Tp, N);
Kr = linspace(4*pi*fstart/c, 4*pi*fstop/c, N);

% sif_mimo(element_index, range_samples) — one row per virtual element
sif_mimo = zeros(total_elements, N);

idx = 0;
for p = 1:NUM_STOPS
    for tx = 1:NUM_TX
        for rx = 1:NUM_RX
            idx = idx + 1;
            xa_pos = all_x_pos(idx);  % this virtual element's x position
            
            sig = zeros(1, N);
            for tgt = 1:size(scene_targets, 1)
                xt = scene_targets(tgt, 1);
                yt = scene_targets(tgt, 2);
                amp = scene_targets(tgt, 3);
                
                R = sqrt((xa_pos - xt)^2 + yt^2);
                tau = 2 * R / c;
                f_beat = cr * tau;
                phase = 2*pi*f_beat*t_sample + 4*pi*fstart*R/c;
                
                sig = sig + amp * exp(1j * phase);
            end
            
            sig = sig + 0.01*(randn(1,N) + 1j*randn(1,N));
            sif_mimo(idx, :) = sig;
        end
    end
end

% Sort data by virtual position (critical for FFT)
sif_mimo = sif_mimo(sort_idx, :);
Xa = all_x_pos_sorted;

fprintf('Data matrix: [%d elements × %d samples]\n\n', size(sif_mimo));

%% RMA Processing (same as Gregory, but with MIMO aperture)
fprintf('========== RMA Processing ==========\n');

sif = sif_mimo;

% Background subtraction
mean_prof = mean(sif, 1);
for ii = 1:size(sif,1)
    sif(ii,:) = sif(ii,:) - mean_prof;
end

% Step 1: Hanning window on range
fprintf('Step 1: Hanning window...\n');
H_range = 0.5 + 0.5*cos(2*pi*((1:N)-N/2)/N);
for ii = 1:size(sif,1)
    sif(ii,:) = sif(ii,:) .* H_range;
end

% Step 2: Along-track FFT with zero-padding
fprintf('Step 2: Along-track FFT...\n');
delta_x = dx_eff;  % effective step = λ/2
zpad = 4096;
szeros = zeros(zpad, N);
index = round((zpad - size(sif,1))/2);
szeros(index+1:index+size(sif,1), :) = sif;
sif = szeros;

S = fftshift(fft(sif, [], 1), 1);
Kx = linspace(-pi/delta_x, pi/delta_x, zpad);

% Plot 2D spectrum
figure('Name', 'MIMO-SAR: 2D Spectrum', 'Position', [50 500 800 400]);
S_img = dbv(S);
imagesc(Kr, Kx, S_img, [max(S_img(:))-40, max(S_img(:))]);
colormap('gray'); colorbar;
xlabel('K_r (rad/m)'); ylabel('K_x (rad/m)');
title('MIMO-SAR: 2D spectrum');

% Step 3: Matched filter
fprintf('Step 3: Matched filter...\n');
phi_mf = zeros(size(S));
for ii = 1:size(S,2)
    for jj = 1:size(S,1)
        if Kr(ii)^2 >= Kx(jj)^2
            phi_mf(jj,ii) = -Rs*Kr(ii) + Rs*sqrt(Kr(ii)^2 - Kx(jj)^2);
        end
    end
end
S_mf = S .* exp(1j * phi_mf);

% Step 4: Stolt interpolation
fprintf('Step 4: Stolt interpolation...\n');
Ky_min = sqrt(min(Kr)^2 - max(abs(Kx))^2);
Ky_max = max(Kr);
if ~isreal(Ky_min), Ky_min = min(Kr)*0.8; end
N_ky = 1024;
Ky_even = linspace(real(Ky_min), Ky_max, N_ky);

S_st = zeros(zpad, N_ky);
for ii = 1:zpad
    Ky_row = sqrt(Kr.^2 - Kx(ii)^2);
    if isreal(Ky_row) && all(Ky_row > 0)
        S_st(ii,:) = interp1(Ky_row, S_mf(ii,:), Ky_even, 'linear', 0);
    end
end
S_st(isnan(S_st)) = 0;

H_ky = 0.5 + 0.5*cos(2*pi*((1:N_ky)-N_ky/2)/N_ky);
for ii = 1:size(S_st,1)
    S_st(ii,:) = S_st(ii,:) .* H_ky;
end

% Step 5: 2D IFFT
fprintf('Step 5: 2D IFFT...\n');
zpad_r = size(S_st,2) * 4;
zpad_cr = size(S_st,1) * 4;
v = ifft2(S_st, zpad_cr, zpad_r);

S_image = v.';

% Axis computation
ky_start = Ky_even(1);
ky_stop  = Ky_even(end);
bw_ky = c * (ky_stop - ky_start) / (4*pi);
max_range_img = c * size(S_st,2) / (2 * bw_ky);

downrange_full  = linspace(0, max_range_img, zpad_r);
max_crossrange  = zpad * delta_x / 2;
crossrange_full = linspace(-max_crossrange, max_crossrange, zpad_cr);

% Auto-calibrate: find brightest pixel in target region
dr_s1 = 1.0; dr_s2 = 6.0; cr_s1 = -1.5; cr_s2 = 1.5;
[~, d1] = min(abs(downrange_full - dr_s1));
[~, d2] = min(abs(downrange_full - dr_s2));
[~, c1] = min(abs(crossrange_full - cr_s1));
[~, c2] = min(abs(crossrange_full - cr_s2));

search_reg = abs(S_image(d1:d2, c1:c2));
[~, mi] = max(search_reg(:));
[pr, pc] = ind2sub(size(search_reg), mi);

peak_dr = downrange_full(d1 + pr - 1);
peak_cr = crossrange_full(c1 + pc - 1);

dr_offset = 2.5 - peak_dr;   % calibrate to known target at y=2.5m
cr_offset = -0.3 - peak_cr;  % calibrate to known target at x=-0.3m

downrange_full  = downrange_full + dr_offset;
crossrange_full = crossrange_full + cr_offset;

fprintf('  Calibration offset: dr=%+.3f m, cr=%+.3f m\n', dr_offset, cr_offset);

% Truncate
dr1 = 1.5; dr2 = 5.0; cr1 = -1.5; cr2 = 1.5;
[~, di1] = min(abs(downrange_full - dr1));
[~, di2] = min(abs(downrange_full - dr2));
[~, ci1] = min(abs(crossrange_full - cr1));
[~, ci2] = min(abs(crossrange_full - cr2));

trunc = S_image(di1:di2, ci1:ci2);
dr_ax = downrange_full(di1:di2);
cr_ax = crossrange_full(ci1:ci2);

% Range compensation
for ii = 1:size(trunc,2)
    trunc(:,ii) = trunc(:,ii) .* max(abs(dr_ax'), 0.1).^(3/2);
end

img_dB = dbv(trunc);

%% Plot MIMO-SAR Image
figure('Name', 'MIMO-SAR Image', 'Position', [100 100 900 700]);
imagesc(cr_ax*100, dr_ax*100, img_dB, [max(img_dB(:))-40 max(img_dB(:))]);
colormap('jet'); colorbar;
xlabel('Cross-range (cm)'); ylabel('Down-range (cm)');
title(sprintf('MIMO-SAR Image — 2×2 sparse, %d stops × %d ch = %d virt elements', ...
    NUM_STOPS, NUM_VIRT, total_elements));
axis equal tight;
set(gca, 'YDir', 'normal');

hold on;
for t = 1:size(scene_targets,1)
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'wo', 'MarkerSize', 16, 'LineWidth', 2);
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'w+', 'MarkerSize', 14, 'LineWidth', 1.5);
end
hold off;

%% Plot without range compensation
figure('Name', 'MIMO-SAR (no comp)', 'Position', [100 50 900 700]);
trunc_nc = S_image(di1:di2, ci1:ci2);
img_nc = dbv(trunc_nc);
imagesc(cr_ax*100, dr_ax*100, img_nc, [max(img_nc(:))-40 max(img_nc(:))]);
colormap('jet'); colorbar;
xlabel('Cross-range (cm)'); ylabel('Down-range (cm)');
title('MIMO-SAR Image — no range compensation');
axis equal tight;
set(gca, 'YDir', 'normal');

hold on;
for t = 1:size(scene_targets,1)
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'wo', 'MarkerSize', 16, 'LineWidth', 2);
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'w+', 'MarkerSize', 14, 'LineWidth', 1.5);
end
hold off;

%% Comparison: single-antenna SAR with same 20 stops
fprintf('\n========== Comparison: Single-Antenna SAR ==========\n');

% Take just the first virtual channel from each stop
sif_single = zeros(NUM_STOPS, N);
for p = 1:NUM_STOPS
    sif_single(p,:) = sif_mimo((p-1)*NUM_VIRT + 1, :);
end

% Same RMA processing
sif_s = sif_single;
mean_s = mean(sif_s,1);
for ii = 1:size(sif_s,1), sif_s(ii,:) = sif_s(ii,:) - mean_s; end
for ii = 1:size(sif_s,1), sif_s(ii,:) = sif_s(ii,:) .* H_range; end

delta_x_single = rail_step;  % 120mm steps
zpad_s = 2048;
sz = zeros(zpad_s, N);
idx_s = round((zpad_s - NUM_STOPS)/2);
sz(idx_s+1:idx_s+NUM_STOPS, :) = sif_s;

S_s = fftshift(fft(sz,[],1),1);
Kx_s = linspace(-pi/delta_x_single, pi/delta_x_single, zpad_s);

phi_s = zeros(size(S_s));
for ii = 1:size(S_s,2)
    for jj = 1:size(S_s,1)
        if Kr(ii)^2 >= Kx_s(jj)^2
            phi_s(jj,ii) = -Rs*Kr(ii) + Rs*sqrt(Kr(ii)^2 - Kx_s(jj)^2);
        end
    end
end
S_s_mf = S_s .* exp(1j*phi_s);

S_s_st = zeros(zpad_s, N_ky);
for ii = 1:zpad_s
    Ky_r = sqrt(Kr.^2 - Kx_s(ii)^2);
    if isreal(Ky_r) && all(Ky_r > 0)
        S_s_st(ii,:) = interp1(Ky_r, S_s_mf(ii,:), Ky_even, 'linear', 0);
    end
end
S_s_st(isnan(S_s_st)) = 0;
for ii = 1:size(S_s_st,1), S_s_st(ii,:) = S_s_st(ii,:) .* H_ky; end

v_s = ifft2(S_s_st, size(S_s_st,1)*4, size(S_s_st,2)*4);
S_s_img = v_s.';

dr_s = linspace(0, max_range_img, size(v_s,2)*1) + dr_offset;
% Use same range axis scaling
dr_s_full = linspace(0, max_range_img, size(S_s_img,1)) + dr_offset;
cr_s_full = linspace(-zpad_s*delta_x_single/2, zpad_s*delta_x_single/2, size(S_s_img,2));

[~, ds1] = min(abs(dr_s_full - dr1));
[~, ds2] = min(abs(dr_s_full - dr2));
[~, cs1] = min(abs(cr_s_full - cr1));
[~, cs2] = min(abs(cr_s_full - cr2));

trunc_s = S_s_img(ds1:ds2, cs1:cs2);
dr_s_ax = dr_s_full(ds1:ds2);
cr_s_ax = cr_s_full(cs1:cs2);

figure('Name', 'Single-Antenna SAR (same stops)', 'Position', [550 100 900 700]);
img_s_dB = dbv(trunc_s);
imagesc(cr_s_ax*100, dr_s_ax*100, img_s_dB, [max(img_s_dB(:))-40 max(img_s_dB(:))]);
colormap('jet'); colorbar;
xlabel('Cross-range (cm)'); ylabel('Down-range (cm)');
title(sprintf('Single-antenna SAR — same %d stops (NO MIMO)', NUM_STOPS));
axis equal tight;
set(gca, 'YDir', 'normal');

hold on;
for t = 1:size(scene_targets,1)
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'wo', 'MarkerSize', 16, 'LineWidth', 2);
    plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
        'w+', 'MarkerSize', 14, 'LineWidth', 1.5);
end
hold off;

%% Summary
fprintf('\n============================================================\n');
fprintf('RESULTS COMPARISON\n');
fprintf('============================================================\n');
fprintf('                    MIMO-SAR        Single-antenna SAR\n');
fprintf('Rail stops:         %d               %d\n', NUM_STOPS, NUM_STOPS);
fprintf('Virtual elements:   %d             %d\n', total_elements, NUM_STOPS);
fprintf('Effective aperture: %.1f cm         %.1f cm\n', total_aperture*100, single_aperture*100);
fprintf('Cross-range res:    %.2f cm         %.2f cm\n', cross_res*100, single_cross_res*100);
fprintf('Range resolution:   %.2f cm         %.2f cm\n', range_res*100, range_res*100);
fprintf('Improvement:        %.1fx better cross-range!\n', single_cross_res/cross_res);
fprintf('\nFigures:\n');
fprintf('  Fig 1: 2D spectrum\n');
fprintf('  Fig 2: MIMO-SAR image (with range compensation)\n');
fprintf('  Fig 3: MIMO-SAR image (no range compensation)\n');
fprintf('  Fig 4: Single-antenna SAR (same stops, for comparison)\n');
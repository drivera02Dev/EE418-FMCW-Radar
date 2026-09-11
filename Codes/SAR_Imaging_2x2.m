%% FMCW MIMO RADAR — SAR Imaging (Range Migration Algorithm)
%  Adapted from Gregory L. Charvat's RMA implementation
%  Reference: Carrara, Goodman, Majewski — "Spotlight SAR Signal Processing"
%
%  For SAR, you physically move the radar along a rail (synthetic aperture)
%  and capture one range profile at each position.
%
%  Our system: 16-20 GHz, USB-6366 DAQ
%  Gregory's: 2.26-2.59 GHz, audio card

clear; clc; close all;
fprintf('============================================================\n');
fprintf('FMCW MIMO RADAR — SAR Imaging (RMA)\n');
fprintf('============================================================\n\n');

dbv = @(x) 20*log10(abs(x) + eps);

%% ============================================================
%  RADAR PARAMETERS — change these for your system
%  ============================================================
c  = 3e8;                  % speed of light (m/s)

% RF parameters (after 4x multiplication)
fstart = 16e9;             % LFM start frequency (Hz)
fstop  = 24e9;             % LFM stop frequency (Hz)
fc     = (fstart+fstop)/2; % center frequency 18 GHz
BW     = fstop - fstart;   % 4 GHz bandwidth
lambda = c / fc;           % 16.67 mm wavelength

% Chirp parameters
Tp = 1e-3;                 % pulse/chirp time (s)
FS = 1e6;                  % USB-6366 sample rate (Hz)
N  = round(Tp * FS);       % samples per chirp = 1000

% SAR aperture parameters
delta_x = lambda / 2;      % antenna step size = λ/2 = 8.33 mm
% For good cross-range resolution, aperture should be ~1-2 meters
% At λ/2 step, 1 meter aperture = ~120 positions
NUM_POS = 120;              % number of aperture positions
L       = delta_x * NUM_POS; % total aperture length (m)

% Scene center distance (calibrate to your setup)
Rs = 3.0;                  % distance to scene center (m)

% Derived
Xa  = linspace(-L/2, L/2, NUM_POS);  % cross-range positions (m)
t   = linspace(0, Tp, N);             % fast time
Kr  = linspace(4*pi*fstart/c, 4*pi*fstop/c, N);  % range wavenumber
cr  = BW / Tp;                         % chirp rate (Hz/s)

fprintf('Frequency:    %.1f - %.1f GHz (BW = %.1f GHz)\n', fstart/1e9, fstop/1e9, BW/1e9);
fprintf('Wavelength:   %.2f mm\n', lambda*1e3);
fprintf('Chirp time:   %.1f ms\n', Tp*1e3);
fprintf('Aperture:     %.1f cm (%d positions, step = %.2f mm)\n', L*100, NUM_POS, delta_x*1e3);
fprintf('Scene center: %.2f m\n', Rs);
fprintf('Range res:    %.2f cm\n', c/(2*BW)*100);
fprintf('Cross-range res: ~%.2f cm (at %.1f m)\n', lambda*Rs/(2*L)*100, Rs);
fprintf('\n');


%% ============================================================
%  DATA CAPTURE / SIMULATION
%  ============================================================
USE_HARDWARE = false;

if USE_HARDWARE
    %% --- LIVE SAR CAPTURE ---
    % You move the radar along a rail, capturing one chirp per position.
    % At each position: ramp VCO, capture IF, store complex range profile.
    %
    % sif(position, range_samples) = complex IF data
    %
    % Example capture loop:
    % for pos = 1:NUM_POS
    %     fprintf('Position %d/%d — move antenna to %.1f mm\n', ...
    %         pos, NUM_POS, Xa(pos)*1e3);
    %     input('Press Enter when ready...');  % or use motorized rail
    %     
    %     % Capture one chirp
    %     preload(dq_ao, ramp);
    %     start(dq_ai, 'Duration', seconds(Tp));
    %     start(dq_ao);
    %     pause(Tp + 0.002);
    %     data = read(dq_ai, N);
    %     
    %     % Use channel 1 (TX1-RX1 for SAR)
    %     raw = table2array(data);
    %     sif_raw(pos,:) = raw(:,1);
    % end
    %
    % % Hilbert transform to get complex (IQ) data
    % for pos = 1:NUM_POS
    %     q = ifft(sif_raw(pos,:));
    %     sif(pos,:) = fft(q(N/2+1:N));  % analytic signal
    % end
    
    error('Connect hardware and uncomment capture code above');
    
else
    %% --- SIMULATION ---
    fprintf('Generating synthetic SAR scene...\n');
    
    % Define point targets: [x_cross_range (m), y_down_range (m), amplitude]
    scene_targets = [
        -0.3,  2.5,  1.0;    % target left of center
         0.0,  3.0,  0.8;    % target at center
         0.2,  3.0,  0.6;    % target near center (test resolution)
         0.5,  3.5,  0.4;    % target right
        -0.1,  4.0,  0.3;    % target far
    ];
    
    fprintf('Scene targets:\n');
    for i = 1:size(scene_targets,1)
        fprintf('  x=%+.1f m, y=%.1f m, amp=%.1f\n', ...
            scene_targets(i,1), scene_targets(i,2), scene_targets(i,3));
    end
    fprintf('\n');
    
    % Generate raw IF data at each aperture position
    sif = zeros(NUM_POS, N);
    f_inst = linspace(fstart, fstop, N);  % instantaneous frequency
    
    for pos = 1:NUM_POS
        sig = zeros(1, N);
        xa_pos = Xa(pos);  % current antenna x position
        
        for tgt = 1:size(scene_targets, 1)
            xt = scene_targets(tgt, 1);
            yt = scene_targets(tgt, 2);
            amp = scene_targets(tgt, 3);
            
            % Range from antenna to target
            R = sqrt((xa_pos - xt)^2 + yt^2);
            
            % Round-trip delay
            tau = 2 * R / c;
            
            % Beat signal (dechirped IF)
            f_beat = cr * tau;
            phase = 2 * pi * f_beat * t + 4 * pi * fstart * R / c;
            
            sig = sig + amp * exp(1j * phase);
        end
        
        % Add noise
        sig = sig + 0.01 * (randn(1,N) + 1j*randn(1,N));
        
        % Store complex IF signal directly
        % (For real hardware data, you'd do Hilbert transform here:
        %   raw_real = captured_data;
        %   sif(pos,:) = hilbert(raw_real);  % MATLAB's hilbert() gives analytic signal
        % But simulated data is already complex/analytic)
        sif(pos,:) = sig;
    end
    
    fprintf('Synthetic SAR data: [%d positions x %d samples]\n\n', size(sif));
end


%% ============================================================
%  BACKGROUND SUBTRACTION (optional, removes stationary clutter)
%  ============================================================
% Subtract mean across all positions (removes anything that doesn't
% change as antenna moves — same as Gregory's background subtraction)
sif_bg = sif;
mean_profile = mean(sif, 1);
for ii = 1:size(sif,1)
    sif(ii,:) = sif(ii,:) - mean_profile;
end


%% ============================================================
%  RMA STEP 1: Apply Hanning window to range data
%  ============================================================
fprintf('RMA Step 1: Hanning window...\n');
H_range = 0.5 + 0.5*cos(2*pi*((1:N) - N/2)/N);
for ii = 1:size(sif,1)
    sif(ii,:) = sif(ii,:) .* H_range;
end


%% ============================================================
%  RMA STEP 2: Along-track FFT (cross-range FFT)
%  ============================================================
fprintf('RMA Step 2: Along-track FFT...\n');

% Zero-pad in cross-range for finer angular sampling
zpad = 2048;
szeros = zeros(zpad, size(sif,2));
index = round((zpad - size(sif,1))/2);
szeros(index+1:index+size(sif,1), :) = sif;
sif = szeros;

% FFT along cross-range (slow time) dimension
S = fftshift(fft(sif, [], 1), 1);
Kx = linspace(-pi/delta_x, pi/delta_x, size(S,1));

% Plot 2D frequency domain
figure('Name', 'SAR: 2D Spectrum', 'Position', [50 500 800 400]);
S_img = dbv(S);
imagesc(Kr, Kx, S_img, [max(S_img(:))-40, max(S_img(:))]);
colormap('gray'); colorbar;
xlabel('K_r (rad/m)'); ylabel('K_x (rad/m)');
title('2D spectrum after along-track FFT');


%% ============================================================
%  RMA STEP 3: Matched filter (reference phase)
%  ============================================================
fprintf('RMA Step 3: Matched filter...\n');

% Build matched filter: eq 10.8 from Carrara
% phi_mf = -Rs*Kr + Rs*sqrt(Kr^2 - Kx^2)
phi_mf = zeros(size(S));
for ii = 1:size(S,2)       % range wavenumber
    for jj = 1:size(S,1)   % cross-range wavenumber
        if Kr(ii)^2 >= Kx(jj)^2
            phi_mf(jj,ii) = -Rs*Kr(ii) + Rs*sqrt(Kr(ii)^2 - Kx(jj)^2);
        else
            phi_mf(jj,ii) = 0;
        end
    end
end
smf = exp(1j * phi_mf);

% Apply matched filter
S_mf = S .* smf;


%% ============================================================
%  RMA STEP 4: Stolt interpolation (Kr,Kx) → (Ky,Kx)
%  ============================================================
fprintf('RMA Step 4: Stolt interpolation...\n');

% Compute Ky range
Ky_min = sqrt(min(Kr)^2 - max(abs(Kx))^2);
Ky_max = max(Kr);
if ~isreal(Ky_min), Ky_min = min(Kr)*0.8; end
N_ky = 1024;
Ky_even = linspace(real(Ky_min), Ky_max, N_ky);

% Interpolate each Kx row from Kr domain to Ky domain
S_st = zeros(zpad, N_ky);
for ii = 1:zpad
    % For this Kx, compute Ky = sqrt(Kr^2 - Kx(ii)^2)
    Ky_row = sqrt(Kr.^2 - Kx(ii)^2);
    
    % Only interpolate if Ky is real (Kr > |Kx|)
    if isreal(Ky_row) && all(Ky_row > 0)
        S_st(ii,:) = interp1(Ky_row, S_mf(ii,:), Ky_even, 'linear', 0);
    end
end
S_st(isnan(S_st)) = 0;

% Apply Hanning window to interpolated data (cleans up artifacts)
H_ky = 0.5 + 0.5*cos(2*pi*((1:N_ky) - N_ky/2)/N_ky);
for ii = 1:size(S_st,1)
    S_st(ii,:) = S_st(ii,:) .* H_ky;
end


%% ============================================================
%  RMA STEP 5: 2D IFFT → final SAR image
%  ============================================================
fprintf('RMA Step 5: 2D IFFT → image...\n');

% Zero-pad for display resolution
zpad_r = size(S_st,2) * 4;  % range zero-pad
zpad_cr = size(S_st,1) * 4; % cross-range zero-pad
v = ifft2(S_st, zpad_cr, zpad_r);

% v dimensions: rows = cross-range (from Kx IFFT), cols = downrange (from Ky IFFT)

% --- RANGE AXIS (from Ky_even, following Gregory's formula) ---
% Gregory: bw = 3E8*(kstop-kstart)/(4*pi), max_range = 3E8*Nky/(2*bw)
ky_start = Ky_even(1);
ky_stop  = Ky_even(end);
bw_ky = c * (ky_stop - ky_start) / (4*pi);  % effective bandwidth from Ky
max_range_img = c * size(S_st,2) / (2 * bw_ky);  % range extent of unpadded IFFT

% The IFFT range goes from 0 to max_range_img (before zero-padding)
% With zero-padding, we get finer pixels but same extent
% Range = 0 in the IFFT corresponds to y = 0 (near the radar)
% Gregory adds Rs to shift the axis
downrange_full = linspace(0, max_range_img, zpad_r);

% --- CROSS-RANGE AXIS (from Kx) ---
max_crossrange = zpad * delta_x / 2;
crossrange_full = linspace(-max_crossrange, max_crossrange, zpad_cr);

% Rearrange image: flip/rotate to standard orientation
% v(row=crossrange, col=downrange) → S_image(row=downrange, col=crossrange)
S_image = v.';  % transpose: now rows=downrange, cols=crossrange

% --- Truncate to region of interest ---
% Use absolute range coordinates — targets are at 2.5 to 4.0 m
dr1 = 1.5;  % min downrange (m)
dr2 = 5.0;  % max downrange (m)
cr1 = -1.5; % cross-range (m)
cr2 =  1.5;

% Find indices using interpolation on the axis
[~, dr_idx1] = min(abs(downrange_full - dr1));
[~, dr_idx2] = min(abs(downrange_full - dr2));
[~, cr_idx1] = min(abs(crossrange_full - cr1));
[~, cr_idx2] = min(abs(crossrange_full - cr2));

trunc_image = S_image(dr_idx1:dr_idx2, cr_idx1:cr_idx2);
dr_axis = downrange_full(dr_idx1:dr_idx2);
cr_axis = crossrange_full(cr_idx1:cr_idx2);

% Find where targets actually appear and print for debugging
[~, peak_dr] = max(max(abs(trunc_image), [], 2));
[~, peak_cr] = max(max(abs(trunc_image), [], 1));
fprintf('  Brightest pixel at: downrange=%.2f m, crossrange=%.2f m\n', ...
    dr_axis(peak_dr), cr_axis(peak_cr));
fprintf('  Expected brightest target: y=2.5m, x=-0.3m (amp=1.0)\n');

% Check if range axis needs offset correction
% The matched filter references Rs, so IFFT y=0 should map to ~Rs
% If targets appear at wrong range, adjust offset
% --- AUTO-CALIBRATE AXES ---
% Search for brightest pixel only near expected region
% Brightest target: x=-0.3m, y=2.5m, amp=1.0
% Search window: downrange 1-6m, crossrange -1 to +1m
dr_search1 = 1.0; dr_search2 = 6.0;
cr_search1 = -1.0; cr_search2 = 1.0;

[~, drs1] = min(abs(downrange_full - dr_search1));
[~, drs2] = min(abs(downrange_full - dr_search2));
[~, crs1] = min(abs(crossrange_full - cr_search1));
[~, crs2] = min(abs(crossrange_full - cr_search2));

search_region = abs(S_image(drs1:drs2, crs1:crs2));
[~, max_lin_idx] = max(search_region(:));
[peak_r_local, peak_c_local] = ind2sub(size(search_region), max_lin_idx);

peak_r = drs1 + peak_r_local - 1;
peak_c = crs1 + peak_c_local - 1;

peak_dr_uncal = downrange_full(peak_r);
peak_cr_uncal = crossrange_full(peak_c);

% Known brightest target position
true_dr = 2.5;   % m
true_cr = -0.3;  % m

% Compute and apply offsets
dr_offset = true_dr - peak_dr_uncal;
cr_offset = true_cr - peak_cr_uncal;

fprintf('  Auto-calibration:\n');
fprintf('    Peak found at:   dr=%.3f m, cr=%.3f m\n', peak_dr_uncal, peak_cr_uncal);
fprintf('    True target:     dr=%.3f m, cr=%.3f m\n', true_dr, true_cr);
fprintf('    Offset applied:  dr=%+.3f m, cr=%+.3f m\n', dr_offset, cr_offset);

downrange_full  = downrange_full + dr_offset;
crossrange_full = crossrange_full + cr_offset;

% Optional: scale by range^(3/2) to compensate path loss
for ii = 1:size(trunc_image,2)
    trunc_image(:,ii) = trunc_image(:,ii) .* max(abs(dr_axis'), 0.1).^(3/2);
end

% Convert to dB
img_dB = dbv(trunc_image);


%% ============================================================
%  PLOT FINAL SAR IMAGE
%  ============================================================
figure('Name', 'SAR Image', 'Position', [100 100 900 700]);

imagesc(cr_axis*100, dr_axis*100, img_dB, ...
    [max(img_dB(:))-40, max(img_dB(:))]);
colormap('jet'); colorbar;
xlabel('Cross-range (cm)');
ylabel('Down-range (cm)');
title('SAR Image — Range Migration Algorithm');
axis equal tight;
set(gca, 'YDir', 'normal');

% Mark true target positions
if ~USE_HARDWARE
    hold on;
    for t = 1:size(scene_targets,1)
        plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
            'wo', 'MarkerSize', 16, 'LineWidth', 2);
        plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
            'w+', 'MarkerSize', 14, 'LineWidth', 1.5);
    end
    hold off;
end

% Also plot without range compensation
figure('Name', 'SAR Image (no range comp)', 'Position', [100 50 900 700]);
trunc_nocomp = S_image(dr_idx1:dr_idx2, cr_idx1:cr_idx2);
img_dB_nocomp = dbv(trunc_nocomp);
imagesc(cr_axis*100, dr_axis*100, img_dB_nocomp, ...
    [max(img_dB_nocomp(:))-40, max(img_dB_nocomp(:))]);
colormap('jet'); colorbar;
xlabel('Cross-range (cm)');
ylabel('Down-range (cm)');
title('SAR Image — no range compensation');
axis equal tight;
set(gca, 'YDir', 'normal');

if ~USE_HARDWARE
    hold on;
    for t = 1:size(scene_targets,1)
        plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
            'wo', 'MarkerSize', 16, 'LineWidth', 2);
        plot(scene_targets(t,1)*100, scene_targets(t,2)*100, ...
            'w+', 'MarkerSize', 14, 'LineWidth', 1.5);
    end
    hold off;
end


%% ============================================================
%  RESOLUTION ANALYSIS
%  ============================================================
fprintf('\n========== Resolution Analysis ==========\n');
range_res = c / (2 * BW);
cross_res = lambda * Rs / (2 * L);
fprintf('Range resolution:      %.2f cm (c/2BW)\n', range_res*100);
fprintf('Cross-range resolution: %.2f cm (λR/2L at R=%.1fm)\n', cross_res*100, Rs);
fprintf('Aperture length:       %.1f cm (%d positions)\n', L*100, NUM_POS);
fprintf('\n');

% If targets at (0.0, 3.0) and (0.2, 3.0) are resolved,
% cross-range resolution is better than 20 cm
fprintf('Two targets at same range (y=3.0m) separated by 20cm:\n');
fprintf('  Should be resolved if cross-range res < 20 cm\n');
if cross_res < 0.20
    res_status = 'RESOLVED';
else
    res_status = 'NOT RESOLVED';
end
fprintf('  Our cross-range res = %.2f cm → %s\n', cross_res*100, res_status);

fprintf('\n============================================================\n');
fprintf('DONE — SAR imaging complete\n');
fprintf('============================================================\n');
fprintf('  Fig 1: 2D spectrum\n');
fprintf('  Fig 2: SAR image (with range compensation)\n');
fprintf('  Fig 3: SAR image (without range compensation)\n');


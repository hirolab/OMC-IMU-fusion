function [cq1, cs1, cwbias1, cabias1, Tw1, Ta1, r1, g131, tshift1, ri1, x, params] = calibrate_OMC_IMU( ...
    time, quat, trans, mtrans, time_imu, w_imu, a_imu)
% [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]
% The smoothing of high frequency data needs a filtering formulation of the
% continuous-time batch estimation, not the global optimization that we use
% for calibration. The difference is that motions during calibration are of
% low frequency, thus, a large time step can be used. 
% 
% addpath('C:\Users\garamizo\Documents\MATLAB\B-splines')

assert(~any(isnan(trans(:))), 'Cannot have gap')

%% Fusing parameters

bord = 4 + 1;
bord_bias = 2 + 1;
dT = 15e-3;
dT_bias = 100e-3;

% Intrinsic params --------------------------------
Nr = ones(1, 3) * 0.4e-3^2;  % from QTM calibration, at 183 Hz
w_white = [0.3573, 0.2735, 0.2739] * 1e-3;
a_white = [0.6213, 0.7069, 0.9750] * 1e-3;
w_walk = [1.2448, 2.2278, 0.1894] * 1e-5;
a_walk = [0.9647, 1.7339, 2.4046] * 1e-6;

Fs_omc = 1 / median(diff(time));
Fs_imu = 1 / median(diff(time_imu));
Nw = (w_white * sqrt(Fs_imu)).^2;
Na = (a_white * sqrt(Fs_imu)).^2;
Nwb = w_walk.^2;
Nab = a_walk.^2;

%% Initialize extrinsic params

[Tw, Ta, wbias, abias, r, g] = calibrate_OMC_IMU_LSQ(...
    time, quat, trans, Fs_omc, time_imu, w_imu, a_imu, Fs_imu);
g13 = g([1,3]);
wbias = wbias(:);
abias = abias(:);
tshift = 0;

nmarkers = size(mtrans, 3);
ri = zeros(3, nmarkers);
for i = 1 : nmarkers
    ri(:,i) = nanmean(quatrotate(quat, mtrans(:,:,i) - trans));
end

params = struct('bord', bord, 'bord_bias', bord_bias, 'dT', dT, 'dT_bias', dT_bias, ...
    'Fs_omc', Fs_omc, 'Fs_imu', Fs_imu, 'Nr', Nr, 'w_white', w_white, 'a_white', a_white, ...
    'w_walk', w_walk, 'a_walk', a_walk, 'Tw', Tw, 'Ta', Ta, 'wbias', wbias, 'abias', abias, ...
    'r', r, 'g13', g13, 'tshift', tshift, 'ri', ri);

%% Initialize motion params
t0 = min(time(1), time_imu(1));
time_norm = (time - t0) / dT;
time2_norm = (time_imu - t0) / dT;
time_norm_bias = (time_imu - t0) / dT_bias;

nknot = ceil(max(time_norm(end), time2_norm(end)) + 1e-10) + bord - 1;
nknot_bias = ceil(max((time(end) - t0) / dT_bias, ...
                      time_norm_bias(end)) + 1e-10) + bord_bias - 1;

% Init orientation ================================
[cq, ~] = bspline_fit_orientation(time_norm, quat, time2_norm, w_imu, Tw, bord, dT, nknot);

% Init translation ================================
[cs, ~] = bspline_fit_translation(time_norm, quat, trans, time2_norm, a_imu, Ta, r, abias, bord, dT, nknot);

% Init biases =====================================
cwbias = repmat(wbias', [nknot_bias, 1]);
cabias = repmat(abias', [nknot_bias, 1]);

%% Optimize

x0 = [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)];
% x0 = [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13];

iters = 30;
X = zeros(iters, length(x0));
Fval = zeros(iters, 1);
x = x0;
for i = 1 : iters
    tic
    [dx, fval] = solve_gauss_newton_sparse_fast(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
        bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab);
    titer = toc;

%     tic
%     [dx, fval] = solve_gauss_newton_sparse(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
%         bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab);
%     titer = toc;


%     tic
%     [dx, fval] = solve_gauss_newton(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
%         bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab);
%     titer = toc;
    
%     a = A - A2;
%     figure, imagesc(a), colorbar()
%     figure, plot(max(a), '.-')

    if any(isnan(dx))
        warning('Went NAN')
        break
    end
    x = x + dx;
    X(i,:) = x;
    Fval(i) = fval;
    fprintf('Iter %2d)\tCost: %.4e\tDuration: %.3f\n', i, fval, titer)
    
    if i > 1 && (Fval(i-1) - fval)/Fval(i-1) < 1/10000
        fprintf('Converged!\n')
        break
    end
end
% figure, plot(X(:,end-10:end) - X(end,end-10:end), '.-')
% figure, plot(X(:,1000) - X(end,1000), '.-')

cq1 = reshape(x(1:3*nknot), [], 3);
cs1 = reshape(x(3*nknot + (1:3*nknot)), [], 3);
cwbias1 = reshape(x(6*nknot + (1:3*nknot_bias)), [], 3);
cabias1 = reshape(x(6*nknot + 3*nknot_bias + (1:3*nknot_bias)), [], 3);
Tw1 = reshape(x(6*nknot + 6*nknot_bias + (1:9)), 3, 3);
Ta1 = reshape(x(6*nknot + 6*nknot_bias + (10:18)), 3, 3);
r1 = reshape(x(6*nknot + 6*nknot_bias + (19:21)), 3, 1);
g131 = reshape(x(6*nknot + 6*nknot_bias + (22:23)), 2, 1);
tshift1 = reshape(x(6*nknot + 6*nknot_bias + 24), 1, 1);
ri1 = reshape(x(6*nknot + 6*nknot_bias + 24 + (1:nmarkers*3)), 3, []);

solve_gauss_newton_sparse_fast(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
        bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab);

end

function [Tw, Ta, wbias, abias, r, g] = calibrate_OMC_IMU_LSQ(time, quat, trans, Fs, time_imu, w_imu, a_imu, Fs_imu)

% [w, wd, ~, a] = QTMParser.body_rates(quat, trans, Fs, [3, 5]);
[wd, w] = angular_rates(quat, Fs, [3, 5]);
[~, ~, a] = deriv_sgolay(trans, Fs, [3, 5]);

% Gyro =============================================
w_imu_interp = interp1(time_imu, w_imu, time);

rows = 10 : size(w,1)-10;  % remove transient errors from derivative
lm1 = fitlm(w(rows,:), w_imu_interp(rows,1), 'RobustOpts', 'on');
lm2 = fitlm(w(rows,:), w_imu_interp(rows,2), 'RobustOpts', 'on');
lm3 = fitlm(w(rows,:), w_imu_interp(rows,3), 'RobustOpts', 'on');

Tw = [lm1.Coefficients.Estimate(2:end)'
       lm2.Coefficients.Estimate(2:end)'
       lm3.Coefficients.Estimate(2:end)'];
wbias = [lm1.Coefficients.Estimate(1), lm2.Coefficients.Estimate(1), lm3.Coefficients.Estimate(1)];

% Acc ================================================
skew_func = @(p) [0, -p(3), p(2)
                  p(3), 0, -p(1)
                  -p(2), p(1), 0];

a_imu_interp = interp1(time_imu, a_imu, time);
R = quat2rotm(quat);

dlen = size(quat, 1);
A = zeros(dlen*3, 18);
b = zeros(dlen*3, 1);

for i = 10 : dlen-10  % avoid transients
    rows = (i-1)*3 + (1:3);
    A(rows,:) = [blkdiag(a_imu_interp(i,:), a_imu_interp(i,:), a_imu_interp(i,:)), ...
                 skew_func(wd(i,:)) + skew_func(w(i,:))*skew_func(w(i,:)), ...
                 R(:,:,i)', eye(3)];
    b(rows) = a(i,:) * R(:,:,i);
end

lm = fitlm(A, b, 'RobustOpts', 'on', 'intercept', false);
% figure, plot(lm)

Ta = inv(reshape(lm.Coefficients.Estimate(1:9), [3, 3])');
r = -lm.Coefficients.Estimate(10:12);
g = lm.Coefficients.Estimate(13:15);
abias = -Ta * lm.Coefficients.Estimate(16:18);

% ====================================================
end


function A = generate_predictor_matrix(bord, time_norm, nknot, order)
    % time_norm := (time - t0) / dT
    % A := length(time_norm) x nknot
    
    % minimum number of knots needed
    nsamples = length(time_norm);
    
    % sparse representation of predictor matrix A
    nonzero_len = ceil(bord / mean(diff(time_norm)));
    aa = zeros(nonzero_len, nknot);
    jj = zeros(nonzero_len, nknot);
    ii = zeros(nonzero_len, nknot);
    
    % single B-spline basis
    ci = zeros(nknot, 1);
    ci(1) = 1;
    
    for j = 1 : nknot

        k0 = find(time_norm >= j, 1, 'first');
        
        if isempty(k0)  % border cases
            k0 = nsamples + 1;
        elseif k0 - nonzero_len < 1
            k0 = nonzero_len + 1;
        end
        
        nonzero_rows = k0 + (-nonzero_len : -1);
        ii(:,j) = nonzero_rows;
        jj(:,j) = j;
        if order == 1
            aa(:,j) = bspline_eval(ci, time_norm(nonzero_rows), bord);
        elseif order == 2
            [~, aa(:,j)] = bspline_eval(ci, time_norm(nonzero_rows), bord);
        elseif order == 3
            [~, ~, aa(:,j)] = bspline_eval(ci, time_norm(nonzero_rows), bord);
        end
        
        ci = circshift(ci, 1);
    end
    
    valid = ii(:) > 0 & jj(:) > 0;
    A = sparse(ii(valid), jj(valid), aa(valid), nsamples, nknot);
end


function [cq, wbias] = bspline_fit_orientation(...
    kk, quat, kk2, wimu, Tw, bord, dt, nknot)

    [q1, q2, q3] = quat2angle(quat, 'YZX');
    q1 = unwrap(q1);
    q = [q3, q1, q2];
    w = wimu * inv(Tw)';

    % position ==============================================
    A1 = generate_predictor_matrix(bord, kk, nknot, 1);
    b1 = q;

    % velocity ================================================
    A2 = generate_predictor_matrix(bord, kk2, nknot, 2) / dt;
    b2 = w;

    % combine ==================================================
    A = [A1, zeros(size(A1,1), 1)
          A2,  ones(size(A2,1), 1)];
    b = [b1; b2];
    W = blkdiag(speye(length(kk)), 1e-8*speye(length(kk2)));  % weight
    cq_bias = (A' * W * A) \ (A' * W * b);
    
    cq = cq_bias(1:end-1,:);
    wbias = cq_bias(end,:) * Tw';
    
    if nargout == 0
        bh = A * cq_bias;
        b1h = bh(1:size(b1,1),:);
        b2h = bh(size(b1,1)+1:end,:);

        fit = 1 - [goodnessOfFit(b1h, b1, 'NMSE'), goodnessOfFit(b2h, b2, 'NMSE')];
        fprintf('Orientation fitness (NMSE): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
            fit(1), fit(2), fit(3), fit(4), fit(5), fit(6))
    
        figure
        h1 = subplot(211); plot(kk, b1, kk, b1h)
        legend([compose("meas %d", 1:size(b1,2)), compose("fit %d", 1:size(b1,2))])
        h2 = subplot(212); plot(kk2, b2, kk2, b2h)
        legend([compose("meas %d", 1:size(b1,2)), compose("fit %d", 1:size(b1,2))])
        linkaxes([h1, h2], 'x')
    end
end

function [cs, grav] = bspline_fit_translation(...
    kk, quat, trans, kk2, aimu, Ta, r, abias, bord, dt, nknot)
    % fit cs as the IMU position

    trans_imu = trans + quatrotate(quatinv(quat), r');
    quat_interp = quatnormalize(interp1(kk, quat, kk2, 'pchip', 'extrap'));
    as = quatrotate(quatinv(quat_interp), (aimu - abias') * inv(Ta)');

    % position ==============================================
    A1 = generate_predictor_matrix(bord, kk, nknot, 1);
    b1 = trans_imu;

    % acceleration ================================================
    A2 = generate_predictor_matrix(bord, kk2, nknot, 3) / dt^2;
    b2 = as;

    % combine ==================================================
    A = [A1, zeros(size(A1,1), 1)
          A2,  ones(size(A2,1), 1)];
    b = [b1; b2];
    W = blkdiag(speye(length(kk)), 1*speye(length(kk2)));  % weight
    cs_grav = (A' * W * A) \ (A' * W * b);
    
    cs = cs_grav(1:end-1,:);
    grav = cs_grav(end,:);
    
    if nargout == 0
        bh = A * cs_grav;
        b1h = bh(1:size(b1,1),:);
        b2h = bh(size(b1,1)+1:end,:);

        fit = 1 - [goodnessOfFit(b1h, b1, 'NMSE'), goodnessOfFit(b2h, b2, 'NMSE')];
        fprintf('Orientation fitness (NMSE): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
            fit(1), fit(2), fit(3), fit(4), fit(5), fit(6))
    
        figure
        h1 = subplot(211); plot(kk, b1, kk, b1h)
        legend([compose("meas %d", 1:size(b1,2)), compose("fit %d", 1:size(b1,2))])
        h2 = subplot(212); plot(kk2, b2, kk2, b2h)
        legend([compose("meas %d", 1:size(b1,2)), compose("fit %d", 1:size(b1,2))])
        linkaxes([h1, h2], 'x')
    end
end


function [dx, cost] = solve_gauss_newton_sparse_fast(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
    bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab)

    integ_npoints = 5;
    
    nmarkers = size(mtrans, 3);
    wh = zeros(length(time2_norm), 3);
    ah = zeros(length(time2_norm), 3);
    mtransh = zeros(length(time_norm), 3, nmarkers);

    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]
    index = reshape((1 : bord)' + (0 : 2)*nknot, [], 1);
    index_bias = reshape((1 : bord_bias)' + (0 : 2)*nknot_bias, [], 1);
    index_imuparam = (1:23)';  % [Tw(:); Ta(:); r; g13]
    index_omcparam_fix = [19:21, 24]';  % r, tshift
    index_omcparam_var = (25:27)';  % ri

    Nw_inv = diag(1./Nw);
    Na_inv = diag(1./Na);
    Nr_inv = diag(1./Nr);
    Nwb_inv = diag(1./Nwb);
    Nab_inv = diag(1./Nab);
    
    A_sparse = zeros(length(x) + ...
        (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2 * (nknot + nknot_bias) + ...
        (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2 * nknot * nmarkers + ...
        (length(index_bias)*2)^2 * (nknot_bias-bord_bias) * integ_npoints, 1);
    i_sparse = ones(length(x) + ...
        (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2 * (nknot + nknot_bias) + ...
        (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2 * nknot * nmarkers + ...
        (length(index_bias)*2)^2 * (nknot_bias-bord_bias) * integ_npoints, 1);
    j_sparse = i_sparse;

    b = zeros(length(x), 1);
    cost = 0;
    
    % Prior ============================================================
    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]

    buf = 1 : length(x);
    A_sparse(buf,:) = [ones(nknot*3,1)/(0.2*pi/180)^2
                    ones(nknot*3,1)/(1e-3)^2
                    ones(nknot_bias*3,1)/(1e-2)^2
                    ones(nknot_bias*3,1)/(1e-5)^2
                    ones(9,1)/(0.1)^2
                    ones(9,1)/(0.1)^2
                    ones(3,1)/(3e-2)^2
                    ones(2,1)/(0.1)^2
                    ones(1,1)/(10e-3 / dT)^2
                    ones(3*nmarkers,1)/(0.5e-2)^2];
    i_sparse(buf) = buf;
    j_sparse(buf) = buf;
    buf = buf + length(x);

    % IMU ==============================================================
    buflen = (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2;
    buf = buf(1) - 1 + (1 : buflen);
    jac = zeros(length(index)*2 + length(index_bias)*2 + length(index_imuparam));
    k0_last = 0; 
    k0_bias_last = 0;
    for k = 1 : length(time2_norm)

        k0 = floor(time2_norm(k));
        k0_bias = floor(time_norm_bias(k));
        
        if k0_last ~= k0 || k0_bias_last ~= k0_bias
            % save progress
            i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
            j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
            A_sparse(buf) = reshape(jac, [], 1);
            buf = buf + buflen;
            jac = jac * 0;
        end

        rows = [k0 + index
                k0 + index + nknot*3
                k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3
                nknot*6 + nknot_bias*6 + index_imuparam];
        vars = x(rows);

        [wh(k,:), ah(k,:), wjac, ajac] = calib_cost_imu(...
            vars.', mod(time2_norm(k), 1), dT, mod(time_norm_bias(k), 1), dT_bias);

        jac = jac + wjac' * Nw_inv * wjac ...
                  + ajac' * Na_inv * ajac;
        b(rows) = b(rows) - wjac' * Nw_inv * (w_imu(k,:) - wh(k,:))' ...
                          - ajac' * Na_inv * (a_imu(k,:) - ah(k,:))';
        cost = cost + (w_imu(k,:) - wh(k,:)) * Nw_inv * (w_imu(k,:) - wh(k,:))' / 2 + ...
                      (a_imu(k,:) - ah(k,:)) * Na_inv * (a_imu(k,:) - ah(k,:))' / 2;
        
        k0_last = k0;
        k0_bias_last = k0_bias;
    end
    i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
    j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
    A_sparse(buf) = reshape(jac, [], 1);
    buf = buf + buflen;

   % OMC ===============================================================
    buflen = (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2;
    buf = buf(1) - 1 + (1 : buflen);
    jac = zeros(length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var));
    k0_last = 0; 
    rows = [];
    for i = 1 : nmarkers
        for k = 1 : length(time_norm)

            if any(isnan(mtrans(k,:,i)))
                continue
            end
            
            k0 = floor(time_norm(k));
            
            if k0_last ~= k0 && ~isempty(rows)
                i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
                j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
                A_sparse(buf) = reshape(jac, [], 1);
                buf = buf + buflen;
                jac = jac * 0;
            end
            
            rows = [k0 + index
                    k0 + index + nknot*3
                    nknot*6 + nknot_bias*6 + index_omcparam_fix
                    nknot*6 + nknot_bias*6 + index_omcparam_var + 3*(i-1)];
            vars = x(rows);

            [mtransh(k,:,i), rjac] = calib_cost_omc(...
                vars.', mod(time_norm(k), 1), dT);
            jac = jac + rjac' * Nr_inv * rjac;
            b(rows) = b(rows) - rjac' * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))';
            cost = cost + (mtrans(k,:,i) - mtransh(k,:,i)) * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))' / 2;
            
            k0_last = k0;
        end
    end
    i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
    j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
    A_sparse(buf) = reshape(jac, [], 1);
    buf = buf + buflen;

    % Bias =============================================================
    % Approximate integral with Euler

    buflen = (length(index_bias)*2)^2;
    buf = buf(1) - 1 + (1 : buflen);
    for k = 1 : nknot_bias-bord_bias

        k0_bias = k - 1;
        rows = [k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3];
        vars = x(rows);
        for kk = 1 : integ_npoints
            t = (kk - 1) / integ_npoints;
            [wbiasd, abiasd, wbiasd_jac, abiasd_jac] = calib_cost_imubias( ...
                vars', t, dT_bias);

            i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
            j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
            A_sparse(buf) = reshape(wbiasd_jac' * Nwb_inv * wbiasd_jac * dT_bias / integ_npoints + ...
                abiasd_jac' * Nab_inv * abiasd_jac * dT_bias / integ_npoints, [], 1);
            buf = buf + buflen;
            
            b(rows) = b(rows) + wbiasd_jac' * Nwb_inv * wbiasd * dT_bias / integ_npoints ...
                              + abiasd_jac' * Nab_inv * abiasd * dT_bias / integ_npoints;
            cost = cost + wbiasd' * Nwb_inv * wbiasd * dT_bias / integ_npoints + ...
                          abiasd' * Nab_inv * abiasd * dT_bias / integ_npoints;
        end
    end
    
    A = sparse(i_sparse, j_sparse, A_sparse, length(x), length(x));
           
    if nargout > 0
        dx = -A \ b;
    
    else
        figure, 
        h1 = subplot(221); plot(time2_norm, w_imu, time2_norm, wh, '--')
        h2 = subplot(223); plot(time2_norm, a_imu, time2_norm, ah, '--')
        
        h3 = subplot(322); plot(time_norm, squeeze(mtrans(:,1,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,1,:)), '--')
        h4 = subplot(324); plot(time_norm, squeeze(mtrans(:,2,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,2,:)), '--')
        h5 = subplot(326); plot(time_norm, squeeze(mtrans(:,3,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,3,:)), '--')
        linkaxes([h1, h2, h3, h4, h5], 'x')
    end
end

function [dx, cost] = solve_gauss_newton_sparse(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
    bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab)

    integ_npoints = 5;
    
    nmarkers = size(mtrans, 3);
    wh = zeros(length(time2_norm), 3);
    ah = zeros(length(time2_norm), 3);
    mtransh = zeros(length(time_norm), 3, nmarkers);

    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]
    index = reshape((1 : bord)' + (0 : 2)*nknot, [], 1);
    index_bias = reshape((1 : bord_bias)' + (0 : 2)*nknot_bias, [], 1);
    index_imuparam = (1:23)';  % [Tw(:); Ta(:); r; g13]
    index_omcparam_fix = [19:21, 24]';  % r, tshift
    index_omcparam_var = (25:27)';  % ri

    Nw_inv = diag(1./Nw);
    Na_inv = diag(1./Na);
    Nr_inv = diag(1./Nr);
    Nwb_inv = diag(1./Nwb);
    Nab_inv = diag(1./Nab);
    
    A_sparse = zeros(length(x) + ...
        (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2 * length(time2_norm) + ...
        (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2 * length(time_norm) * nmarkers + ...
        (length(index_bias)*2)^2 * (nknot_bias-bord_bias) * integ_npoints, 1);
    i_sparse = ones(length(x) + ...
        (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2 * length(time2_norm) + ...
        (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2 * length(time_norm) * nmarkers + ...
        (length(index_bias)*2)^2 * (nknot_bias-bord_bias) * integ_npoints, 1);
    j_sparse = i_sparse;

    b = zeros(length(x), 1);
    cost = 0;
    
    % Prior ============================================================
    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]

    buf = 1 : length(x);
    A_sparse(buf,:) = [ones(nknot*3,1)/(0.2*pi/180)^2
                    ones(nknot*3,1)/(1e-3)^2
                    ones(nknot_bias*3,1)/(1e-5)^2
                    ones(nknot_bias*3,1)/(1e-5)^2
                    ones(9,1)/(0.1)^2
                    ones(9,1)/(0.1)^2
                    ones(3,1)/(3e-2)^2
                    ones(2,1)/(0.1)^2
                    ones(1,1)/(10e-3 / dT)^2
                    ones(3*nmarkers,1)/(0.5e-2)^2];
    i_sparse(buf) = buf;
    j_sparse(buf) = buf;
    buf = buf + length(x);

    % IMU ==============================================================
    buflen = (length(index)*2 + length(index_bias)*2 + length(index_imuparam))^2;
    buf = buf(1) - 1 + (1 : buflen);
    for k = 1 : length(time2_norm)

        k0 = floor(time2_norm(k));
        k0_bias = floor(time_norm_bias(k));
        rows = [k0 + index
                k0 + index + nknot*3
                k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3
                nknot*6 + nknot_bias*6 + index_imuparam];
        vars = x(rows);

        [wh(k,:), ah(k,:), wjac, ajac] = calib_cost_imu(...
            vars.', mod(time2_norm(k), 1), dT, mod(time_norm_bias(k), 1), dT_bias);

        i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
        j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
        A_sparse(buf) = reshape(wjac' * Nw_inv * wjac + ajac' * Na_inv * ajac, [], 1);
        buf = buf + buflen;
        
        b(rows) = b(rows) - wjac' * Nw_inv * (w_imu(k,:) - wh(k,:))' - ajac' * Na_inv * (a_imu(k,:) - ah(k,:))';
        cost = cost + (w_imu(k,:) - wh(k,:)) * Nw_inv * (w_imu(k,:) - wh(k,:))' / 2 + ...
            (a_imu(k,:) - ah(k,:)) * Na_inv * (a_imu(k,:) - ah(k,:))' / 2;
    end

   % OMC ===============================================================
    buflen = (length(index)*2 + length(index_omcparam_fix) + length(index_omcparam_var))^2;
    buf = buf(1) - 1 + (1 : buflen);
    for i = 1 : nmarkers
        for k = 1 : length(time_norm)

            if isnan(mtrans(k,1,i))
                continue
            end
            
            k0 = floor(time_norm(k));
            rows = [k0 + index
                    k0 + index + nknot*3
                    nknot*6 + nknot_bias*6 + index_omcparam_fix
                    nknot*6 + nknot_bias*6 + index_omcparam_var + 3*(i-1)];
            vars = x(rows);

            [mtransh(k,:,i), rjac] = calib_cost_omc(...
                vars.', mod(time_norm(k), 1), dT);

            i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
            j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
            A_sparse(buf) = reshape(rjac' * Nr_inv * rjac, [], 1);
            buf = buf + buflen;
            
            b(rows) = b(rows) - rjac' * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))';
            cost = cost + (mtrans(k,:,i) - mtransh(k,:,i)) * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))' / 2;
        end
    end

    % Bias =============================================================
    % Approximate integral with Euler

    buflen = (length(index_bias)*2)^2;
    buf = buf(1) - 1 + (1 : buflen);
    for k = 1 : nknot_bias-bord_bias

        k0_bias = k - 1;
        rows = [k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3];
        vars = x(rows);
        for kk = 1 : integ_npoints
            t = (kk - 1) / integ_npoints;
            [wbiasd, abiasd, wbiasd_jac, abiasd_jac] = calib_cost_imubias( ...
                vars', t, dT_bias);

            i_sparse(buf) = reshape(repmat(rows, [1, length(rows)]), [], 1);
            j_sparse(buf) = reshape(repmat(rows', [length(rows), 1]), [], 1);
            A_sparse(buf) = reshape(wbiasd_jac' * Nwb_inv * wbiasd_jac * dT_bias / integ_npoints + ...
                abiasd_jac' * Nab_inv * abiasd_jac * dT_bias / integ_npoints, [], 1);
            buf = buf + buflen;
            
            b(rows) = b(rows) + wbiasd_jac' * Nwb_inv * wbiasd * dT_bias / integ_npoints ...
                              + abiasd_jac' * Nab_inv * abiasd * dT_bias / integ_npoints;
            cost = cost + wbiasd' * Nwb_inv * wbiasd * dT_bias / integ_npoints + ...
                          abiasd' * Nab_inv * abiasd * dT_bias / integ_npoints;
        end
    end
    
    A = sparse(i_sparse, j_sparse, A_sparse, length(x), length(x));
           
    if nargout > 0
        dx = -A \ b;
    
    else
        figure, 
        h1 = subplot(221); plot(time2_norm, w_imu, time2_norm, wh, '--')
        h2 = subplot(223); plot(time2_norm, a_imu, time2_norm, ah, '--')
        
        h3 = subplot(322); plot(time_norm, squeeze(mtrans(:,1,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,1,:)), '--')
        h4 = subplot(324); plot(time_norm, squeeze(mtrans(:,2,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,2,:)), '--')
        h5 = subplot(326); plot(time_norm, squeeze(mtrans(:,3,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,3,:)), '--')
        linkaxes([h1, h2, h3, h4, h5], 'x')
    end
end


function [dx, cost] = solve_gauss_newton(x, time_norm, mtrans, time2_norm, w_imu, a_imu, time_norm_bias, ...
    bord, bord_bias, dT, dT_bias, nknot, nknot_bias, Nw, Na, Nr, Nwb, Nab)

    nmarkers = size(mtrans, 3);
    wh = zeros(length(time2_norm), 3);
    ah = zeros(length(time2_norm), 3);
    mtransh = zeros(length(time_norm), 3, nmarkers);

    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]
    index = reshape((1 : bord)' + (0 : 2)*nknot, [], 1);
    index_bias = reshape((1 : bord_bias)' + (0 : 2)*nknot_bias, [], 1);
    index_imuparam = (1:23)';  % [Tw(:); Ta(:); r; g13]
    index_omcparam_fix = [19:21, 24]';  % r, tshift
    index_omcparam_var = (25:27)';  % ri

    Nw_inv = diag(1./Nw);
    Na_inv = diag(1./Na);
    Nr_inv = diag(1./Nr);
    Nwb_inv = diag(1./Nwb);
    Nab_inv = diag(1./Nab);

    b = zeros(length(x), 1);
    cost = 0;
    
    % Prior ============================================================
    % [cq(:); cs(:); cwbias(:); cabias(:); Tw(:); Ta(:); r; g13; tshift; ri(:)]
    A = blkdiag(eye(nknot*3)/(0.2*pi/180)^2, ...
                    eye(nknot*3)/(1e-3)^2, ...
                    eye(nknot_bias*3)/(1e-5)^2, ...
                    eye(nknot_bias*3)/(1e-5)^2, ...
                    eye(9)/(0.1)^2, ...
                    eye(9)/(0.1)^2, ...
                    eye(3)/(3e-2)^2, ...
                    eye(2)/(0.1)^2, ...
                    eye(1)/(10e-3 / dT)^2, ...
                    eye(3*nmarkers)/(0.5e-2)^2);

    % IMU ==============================================================
    for k = 1 : length(time2_norm)

        k0 = floor(time2_norm(k));
        k0_bias = floor(time_norm_bias(k));
        rows = [k0 + index
                k0 + index + nknot*3
                k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3
                nknot*6 + nknot_bias*6 + index_imuparam];
        vars = x(rows);

        [wh(k,:), ah(k,:), wjac, ajac] = calib_cost_imu(...
            vars.', mod(time2_norm(k), 1), dT, mod(time_norm_bias(k), 1), dT_bias);

        A(rows,rows) = A(rows,rows) + wjac' * Nw_inv * wjac + ajac' * Na_inv * ajac;
        b(rows) = b(rows) - wjac' * Nw_inv * (w_imu(k,:) - wh(k,:))' - ajac' * Na_inv * (a_imu(k,:) - ah(k,:))';
        cost = cost + (w_imu(k,:) - wh(k,:)) * Nw_inv * (w_imu(k,:) - wh(k,:))' / 2 + ...
            (a_imu(k,:) - ah(k,:)) * Na_inv * (a_imu(k,:) - ah(k,:))' / 2;
    end

   % OMC ===============================================================
    for k = 1 : length(time_norm)

        k0 = floor(time_norm(k));
        for i = 1 : nmarkers
            if any(isnan(mtrans(k,:,i)))
                continue
            end
            
            rows = [k0 + index
                    k0 + index + nknot*3
                    nknot*6 + nknot_bias*6 + index_omcparam_fix
                    nknot*6 + nknot_bias*6 + index_omcparam_var + 3*(i-1)];
            vars = x(rows);

            [mtransh(k,:,i), rjac] = calib_cost_omc(...
                vars.', mod(time_norm(k), 1), dT);

            A(rows,rows) = A(rows,rows) + rjac' * Nr_inv * rjac;
            b(rows) = b(rows) - rjac' * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))';
            cost = cost + (mtrans(k,:,i) - mtransh(k,:,i)) * Nr_inv * (mtrans(k,:,i) - mtransh(k,:,i))' / 2;
        end
    end

    % Bias =============================================================
    % Approximate integral with Euler
    integ_npoints = 5;

    for k = 1 : nknot_bias-bord_bias

        k0_bias = k - 1;
        rows = [k0_bias + index_bias + nknot*6
                k0_bias + index_bias + nknot*6 + nknot_bias*3];
        vars = x(rows);
        for kk = 1 : integ_npoints
            t = (kk - 1) / integ_npoints;
            [wbiasd, abiasd, wbiasd_jac, abiasd_jac] = calib_cost_imubias( ...
                vars', t, dT_bias);

            A(rows,rows) = A(rows,rows) + wbiasd_jac' * Nwb_inv * wbiasd_jac * dT_bias / integ_npoints + ...
                abiasd_jac' * Nab_inv * abiasd_jac * dT_bias / integ_npoints;
            b(rows) = b(rows) + wbiasd_jac' * Nwb_inv * wbiasd * dT_bias / integ_npoints ...
                              + abiasd_jac' * Nab_inv * abiasd * dT_bias / integ_npoints;
            cost = cost + wbiasd' * Nwb_inv * wbiasd * dT_bias / integ_npoints + ...
                          abiasd' * Nab_inv * abiasd * dT_bias / integ_npoints;
        end
    end

    if nargout > 0
        dx = -A \ b;
    
    else
        figure, 
        h1 = subplot(221); plot(time2_norm, w_imu, time2_norm, wh, '--')
        h2 = subplot(223); plot(time2_norm, a_imu, time2_norm, ah, '--')
        
        h3 = subplot(322); plot(time_norm, squeeze(mtrans(:,1,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,1,:)), '--')
        h4 = subplot(324); plot(time_norm, squeeze(mtrans(:,2,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,2,:)), '--')
        h5 = subplot(326); plot(time_norm, squeeze(mtrans(:,3,:)))
        hold on, set(gca, 'ColorOrderIndex', 1), plot(time_norm, squeeze(mtransh(:,3,:)), '--')
        linkaxes([h1, h2, h3, h4, h5], 'x')
    end
end


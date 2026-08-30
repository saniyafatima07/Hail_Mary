function paper_virtual_target_demo(num_runs)
% =========================================================================
% PAPER-BASED DISPERSION / CORRECTION / VIRTUAL-TARGET DEMONSTRATION
%
% This is an ABSTRACT 2-D statistical simulation.
%
% It demonstrates:
%   1. Monte Carlo uncontrolled impact dispersion
%   2. 3-sigma dispersion ellipse
%   3. Abstract correction-capability ellipse
%   4. Ellipse overlap
%   5. Virtual-target geometry
%   6. Comparison of uncontrolled and corrected distributions
%
% It does NOT model real artillery dynamics, fin commands, or weapon
% guidance/control.
% =========================================================================

    if nargin < 1
        num_runs = 10000;
    end

    rng(42);       % Reproducible Monte Carlo experiment

    fprintf('\n==============================================\n');
    fprintf(' PAPER-BASED DISPERSION ANALYSIS\n');
    fprintf('==============================================\n');
    fprintf('Monte Carlo samples : %d\n\n', num_runs);

    %% ================================================================
    % 1. ABSTRACT TARGET
    % ================================================================

    target = [0, 0];

    %% ================================================================
    % 2. UNCONTROLLED DISPERSION MODEL
    %
    % The Monte Carlo samples represent the impact uncertainty of an
    % abstract projectile.
    %
    % X -> downrange-like direction
    % Z -> crossrange-like direction
    % ================================================================

    sigma_x = 80.0;       % abstract 1-sigma longitudinal uncertainty
    sigma_z = 35.0;       % abstract 1-sigma lateral uncertainty

    % Optional correlation between X and Z.
    correlation = 0.15;

    covariance = [ ...
        sigma_x^2, correlation*sigma_x*sigma_z;
        correlation*sigma_x*sigma_z, sigma_z^2 ...
    ];

    % Generate standard normal samples
    standard_samples = randn(num_runs, 2);

    % Convert them to correlated samples
    L = chol(covariance, 'lower');

    uncontrolled_error = standard_samples * L';

    uncontrolled_points = uncontrolled_error + target;

    %% ================================================================
    % 3. ESTIMATE MONTE CARLO STATISTICS
    % ================================================================

    mean_point = mean(uncontrolled_points, 1);

    dx = uncontrolled_points(:,1) - target(1);
    dz = uncontrolled_points(:,2) - target(2);

    miss_distance = sqrt(dx.^2 + dz.^2);

    measured_sigma_x = std(dx);
    measured_sigma_z = std(dz);

    cep50 = prctile(miss_distance, 50);
    radius90 = prctile(miss_distance, 90);

    fprintf('Measured sigma_x     : %.2f\n', measured_sigma_x);
    fprintf('Measured sigma_z     : %.2f\n', measured_sigma_z);
    fprintf('CEP (50%% radial)     : %.2f\n', cep50);
    fprintf('90%% radial radius    : %.2f\n', radius90);
    fprintf('Mean impact point    : [%.2f, %.2f]\n\n', ...
            mean_point(1), mean_point(2));

    %% ================================================================
    % 4. PAPER-STYLE 3-SIGMA DISPERSION ELLIPSE
    %
    % Approximate form:
    %
    %       x^2             z^2
    %       ---- + --------- = 1
    %      (3sx)^2         (3sz)^2
    %
    % ================================================================

    sigma_boundary = 3.0;

    semi_x = sigma_boundary * measured_sigma_x;
    semi_z = sigma_boundary * measured_sigma_z;

    theta = linspace(0, 2*pi, 500);

    dispersion_x = target(1) + semi_x*cos(theta);
    dispersion_z = target(2) + semi_z*sin(theta);

    %% ================================================================
    % 5. ABSTRACT CORRECTION-CAPABILITY REGION
    %
    % This is deliberately an abstract ellipse.
    %
    % The ellipse represents the region of errors that the abstract
    % correction mechanism is assumed capable of compensating.
    % ================================================================

    correction_a = 105.0;
    correction_b = 45.0;

    correction_center = [35.0, 0.0];

    correction_x = correction_center(1) + correction_a*cos(theta);
    correction_z = correction_center(2) + correction_b*sin(theta);

    %% ================================================================
    % 6. VIRTUAL-TARGET GEOMETRY
    %
    % We determine the displacement between the centers of the two
    % regions.
    %
    % This is the geometric quantity used by the paper's virtual-target
    % construction.
    % ================================================================

    center_difference = correction_center - target;

    center_distance = norm(center_difference);

    fprintf('Dispersion ellipse semi-axis X : %.2f\n', semi_x);
    fprintf('Dispersion ellipse semi-axis Z : %.2f\n', semi_z);
    fprintf('Correction ellipse semi-axis A : %.2f\n', correction_a);
    fprintf('Correction ellipse semi-axis B : %.2f\n', correction_b);
    fprintf('Center separation              : %.2f\n\n', center_distance);

    %% ================================================================
    % 7. SAMPLE THE DISPERSION ELLIPSE
    %
    % Instead of solving the full ellipse-intersection equations
    % symbolically, sample the ellipse boundary and determine how much
    % of the uncontrolled region lies inside the correction region.
    %
    % This is useful for visualizing the geometry.
    % ================================================================

    correction_test = ...
        ((dispersion_x - correction_center(1)).^2 / correction_a^2) + ...
        ((dispersion_z - correction_center(2)).^2 / correction_b^2);

    boundary_inside = correction_test <= 1;

    overlap_fraction_boundary = mean(boundary_inside);

    fprintf('Boundary overlap fraction : %.3f\n', ...
            overlap_fraction_boundary);

    %% ================================================================
    % 8. ABSTRACT VIRTUAL TARGET
    %
    % For this demonstration, move the virtual target toward the region
    % where the two regions overlap.
    %
    % This is a geometric/statistical construction, not a flight-control
    % command.
    % ================================================================

    if center_distance > 0

        direction_to_correction = ...
            center_difference / center_distance;

        % Abstract weighting factor.
        %
        % 0 -> physical target
        % 1 -> correction-region center
        %
        % We choose an intermediate point to demonstrate the geometry.

        virtual_target_fraction = 0.50;

        virtual_target = target + ...
            virtual_target_fraction * ...
            center_difference;

    else

        virtual_target = target;

    end

    fprintf('Physical target : [%.2f, %.2f]\n', ...
            target(1), target(2));

    fprintf('Virtual target  : [%.2f, %.2f]\n\n', ...
            virtual_target(1), virtual_target(2));

    %% ================================================================
    % 9. SHIFTED / ABSTRACT CORRECTED DISTRIBUTION
    %
    % This section is purely statistical.
    %
    % We shift the error distribution toward the virtual-target geometry
    % to demonstrate how the final impact cloud changes.
    % ================================================================

    correction_factor = 0.50;

    corrected_error = uncontrolled_error;

    corrected_error(:,1) = ...
        corrected_error(:,1) - ...
        correction_factor * virtual_target(1);

    corrected_error(:,2) = ...
        corrected_error(:,2) - ...
        correction_factor * virtual_target(2);

    corrected_points = corrected_error + target;

    %% ================================================================
    % 10. CORRECTED STATISTICS
    % ================================================================

    corrected_dx = corrected_points(:,1) - target(1);
    corrected_dz = corrected_points(:,2) - target(2);

    corrected_distance = ...
        sqrt(corrected_dx.^2 + corrected_dz.^2);

    corrected_cep50 = prctile(corrected_distance, 50);
    corrected_radius90 = prctile(corrected_distance, 90);

    corrected_mip = mean(corrected_points, 1);

    fprintf('------------- COMPARISON -------------\n');
    fprintf('                         Unguided      Abstract corrected\n');
    fprintf('CEP (50%%)                %8.2f        %8.2f\n', ...
            cep50, corrected_cep50);

    fprintf('90%% radial radius        %8.2f        %8.2f\n', ...
            radius90, corrected_radius90);

    fprintf('MIP X                     %8.2f        %8.2f\n', ...
            mean_point(1), corrected_mip(1));

    fprintf('MIP Z                     %8.2f        %8.2f\n', ...
            mean_point(2), corrected_mip(2));

    cep_improvement = ...
        100 * (cep50 - corrected_cep50) / cep50;

    fprintf('\nRelative CEP change      : %.2f %%\n', ...
            cep_improvement);

    fprintf('==============================================\n');

    %% ================================================================
    % 11. PLOT
    % ================================================================

    figure('Color', 'w', ...
           'Position', [100 100 1200 750], ...
           'Name', 'Paper-Based Dispersion Analysis');

    hold on;
    grid on;
    axis equal;

    % ------------------------------------------------
    % Uncontrolled impact points
    % ------------------------------------------------

    scatter(uncontrolled_points(:,1), ...
            uncontrolled_points(:,2), ...
            8, ...
            'filled', ...
            'DisplayName', 'Uncontrolled impacts');

    % ------------------------------------------------
    % Corrected statistical distribution
    % ------------------------------------------------

    scatter(corrected_points(:,1), ...
            corrected_points(:,2), ...
            8, ...
            'filled', ...
            'DisplayName', 'Abstract corrected impacts');

    % ------------------------------------------------
    % Physical target
    % ------------------------------------------------

    plot(target(1), target(2), ...
         'kx', ...
         'MarkerSize', 14, ...
         'LineWidth', 3, ...
         'DisplayName', 'Physical target');

    % ------------------------------------------------
    % Virtual target
    % ------------------------------------------------

    plot(virtual_target(1), ...
         virtual_target(2), ...
         'ko', ...
         'MarkerSize', 10, ...
         'LineWidth', 2, ...
         'DisplayName', 'Virtual target');

    % ------------------------------------------------
    % Mean impact point
    % ------------------------------------------------

    plot(mean_point(1), ...
         mean_point(2), ...
         'ks', ...
         'MarkerSize', 9, ...
         'MarkerFaceColor', 'k', ...
         'DisplayName', 'Uncontrolled MIP');

    % ------------------------------------------------
    % Dispersion ellipse
    % ------------------------------------------------

    plot(dispersion_x, ...
         dispersion_z, ...
         '--', ...
         'LineWidth', 2, ...
         'DisplayName', '3\sigma dispersion ellipse');

    % ------------------------------------------------
    % Correction ellipse
    % ------------------------------------------------

    plot(correction_x, ...
         correction_z, ...
         '-.', ...
         'LineWidth', 2, ...
         'DisplayName', 'Correction-capability ellipse');

    % ------------------------------------------------
    % Target -> virtual target
    % ------------------------------------------------

    plot([target(1), virtual_target(1)], ...
         [target(2), virtual_target(2)], ...
         'k:', ...
         'LineWidth', 1.5, ...
         'DisplayName', 'Virtual-target displacement');

    xlabel('X error / longitudinal direction');
    ylabel('Z error / lateral direction');

    title('Dispersion, Correction Region and Virtual Target');

    legend('Location', 'best');

    %% ================================================================
    % 12. SECOND FIGURE: RADIAL ERROR DISTRIBUTIONS
    % ================================================================

    figure('Color', 'w', ...
           'Position', [150 150 900 600], ...
           'Name', 'Radial Error Comparison');

    histogram(miss_distance, ...
              50, ...
              'Normalization', 'pdf', ...
              'DisplayName', 'Uncontrolled');

    hold on;
    grid on;

    histogram(corrected_distance, ...
              50, ...
              'Normalization', 'pdf', ...
              'DisplayName', 'Abstract corrected');

    xline(cep50, ...
          '--', ...
          'LineWidth', 2, ...
          'DisplayName', sprintf('Unguided CEP = %.1f', cep50));

    xline(corrected_cep50, ...
          '-.', ...
          'LineWidth', 2, ...
          'DisplayName', sprintf('Corrected CEP = %.1f', corrected_cep50));

    xlabel('Radial error');
    ylabel('Probability density');

    title('Monte Carlo Error Distribution');

    legend('Location', 'best');

end
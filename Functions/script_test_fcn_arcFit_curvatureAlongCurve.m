%% script_test_fcn_arcFit_curvatureAlongCurve
% Exercises the function: fcn_arcFit_curvatureAlongCurve

% 2024_06_27 - S. Brennan
% -- wrote the code

close all;

flag_be_verbose = 1;

%% Detailed example 
% 
% Fill Data
rng(1); % Fix the random number, for debugging

% Set 1==1 to use test track data. Set 1==0 to use artificial data.
if 1==0
    % TEST TRACK DATA

    % Check to see if XY data for the centerline of the original track lane was loaded earlier
    mat_filename = fullfile(cd,'Data','Centerline_OriginalTrackLane_InnerMarkerClusterCenterOfDoubleYellow.mat');
    if exist(mat_filename,'file')
        load(mat_filename,'XY_data');
    end

    % Pre-append and post-append data, to allow wrap-around?
    % ADD THIS?

    % Since the XY data is very dense, keep only 1 of every "keep_every" points
    keep_every = 20; % 20 works OK, 5 does not work
    indicies = (1:length(XY_data(:,1)))';
    small_XY_data_indicies = find(0==mod(indicies,keep_every));
    small_XY_data = XY_data(small_XY_data_indicies,:);
    points_to_fit = small_XY_data;
else
    % ARTIFICIAL ARC DATA

    % arc_pattern has [1/R and L] for each segment as a row
    arc_pattern = [...
        1/20, 15;
        0 20;
        -1/5 10;
        0 10;
        1/15 40;
        0 15
        -1/10 20];

    % arc_pattern = [...
    %     1/20, 45;
    %     0 20;
    %     -1/10 10;
    %     0 10];

    M = 5; % How many points per meter
    sigma = 0.02; % The standard deviation in the points relative to the perfect function fit, in meters

    [points_to_fit, ~, ~, trueArcStartIndicies, trueNamedCurveTypes, trueParameters] = fcn_geometry_fillArcSequenceTestPoints(arc_pattern, M, sigma, -1);

end

% Add the tiniest bit of noise - this prevents singularities when doing
% regressions
points_to_fit = points_to_fit + 0.001*rand(size(points_to_fit));

% Plot the raw data
fig_num = 1;
figure(fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

plot(points_to_fit(:,1),points_to_fit(:,2),'k.','MarkerSize',20);

%%%% Assess the data properties
% to find average spacing, number of points, etc.
Npoints = length(points_to_fit(:,1));
minimum_island_separation = 10; % Units are meters. See comments below for explanation

spatial_differences = diff(points_to_fit(:,1:2));
spatial_distances   = real(sum(spatial_differences.^2,2).^0.5);
average_spacing     = mean(spatial_distances);

% Check if the data is a loop (this changes a few steps that follow)
distance_XY_from_start = real(sum((points_to_fit(:,1:2) - points_to_fit(1,1:2)).^2,2).^0.5);
max_distance_XY_from_start = max(distance_XY_from_start);
distance_start_to_end  = real(sum((points_to_fit(end,1:2) - points_to_fit(1,1:2)).^2,2).^0.5);
flag_is_a_loop = 0;

% Make sure the start/end are not 5 standard deviations away from each
% other, and no more than 2 island separations (e.g. there's a chance they
% are connected to each other).
if distance_start_to_end<(5*average_spacing) || distance_start_to_end<(2*minimum_island_separation)
    flag_is_a_loop = 1;
end

if 1==flag_be_verbose
    % Report results
    fprintf(1,'\n\n');
    fprintf(1,'RAW DATA ASSESSMENT:\n');
    fprintf(1,'Number of points: %.0f\n',Npoints);
    fprintf(1,'Spacing:\n');
    fprintf(1,'Mean spacing: %.3f meters\n',average_spacing);
    fprintf(1,'Stdv spacing: %.3f meters\n',std(spatial_distances));
    fprintf(1,'Max  spacing: %.3f meters\n',max(spatial_distances));
    fprintf(1,'Min  spacing: %.3f meters\n',min(spatial_distances));
    fprintf(1,'Looping:\n');
    fprintf(1,'Max data distance from start: %.3f meters\n',max_distance_XY_from_start);
    fprintf(1,'Distance from start to end: %.3f meters\n',distance_start_to_end);
    fprintf(1,'Is this data a loop? %.0f \n',flag_is_a_loop);
end



%%% Show how to use fcn_arcFit_curvatureAlongCurve to find any islands in geometric information
% Islands are where there are interconnected arcs that have no line
% segments within. Islands are useful for analysis because calculations
% done within an island are not affected by calculations in other islands,
% and so the data can be sub-grouped by island and processed island by
% island, thereby saving huge amounts of computation.
%
% We find islands by calculating if there are "gaps" in the
% data wherein the curvature of the data, within the gaps, is
% indistinguishable from a line fit. This calculation is within the
% curvatureAtPoint function. The curvature calculation is VERY slow as it
% checks, via regression, all possible circles that can be created with the
% given point at the center of the data set. One can set number of data
% points to consider to right and left of the test point. Default is to use
% them all, but for large data this is VERY slow. Instead of using them
% all, we can specify the minimum distance ever expected between islands,
% e.g. the minimum distance allowed to be a "straightaway" on a road. This
% is a user-defined parameter that of course depends on the actual road.
% From this, we can calculate how many data points should be used for
% curvature calculations. Of note: data_width needs to be at least 2 or
% more for the curvature calculation to work.

% % Below is some code that can auto-calculate data widths - this could be
% % improved.
% % Set up the distances wherein the islands must be separated to be an
% % "island"
% if Npoints>100
%     data_width = ceil(minimum_island_separation/average_spacing);
%     if data_width<=2
%         data_width = [];
%         warning('Not enough data for curvature calculations. Using all data.');
%     end
% 
%     % Enforce a minimum number of points, otherwise the SNR is very poor- we do not want to use less than
%     % 20 points typically
%     data_width = max(data_width,20);
% end

fig_num = 1234;
data_width = 200;
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(points_to_fit, (data_width), (fig_num));


%% Example of Arc-Segment-Arc with long segment
 
% Fill Data
rng(1); % Fix the random number, for debugging

% arc_pattern has [1/R and L] for each segment as a row
arc_pattern = [...
    0 20;
    1/20, 30;
    0 20;
    1/20 30;
    0 10];

M = 5; % How many points per meter
sigma = 0.02; % The standard deviation in the points relative to the perfect function fit, in meters

[points_to_fit, ~, ~, trueArcStartIndicies, trueNamedCurveTypes, trueParameters] = fcn_geometry_fillArcSequenceTestPoints(arc_pattern, M, sigma, -1);

%%%% Plot the raw data
fig_num = 1;
figure(fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

plot(points_to_fit(:,1),points_to_fit(:,2),'k.','MarkerSize',20);

fig_num = 1111;
data_width = 200;
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(points_to_fit, (data_width), (fig_num));

%% Example of Arc-Segment-Arc with short segment
 
% Fill Data
rng(1); % Fix the random number, for debugging

% arc_pattern has [1/R and L] for each segment as a row
arc_pattern = [...
    0 20;
    1/20, 30;
    0 5;
    1/20 30;
    0 10];

M = 5; % How many points per meter
sigma = 0.02; % The standard deviation in the points relative to the perfect function fit, in meters

[points_to_fit, ~, ~, trueArcStartIndicies, trueNamedCurveTypes, trueParameters] = fcn_geometry_fillArcSequenceTestPoints(arc_pattern, M, sigma, -1);

%%%% Plot the raw data
fig_num = 1;
figure(fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

plot(points_to_fit(:,1),points_to_fit(:,2),'k.','MarkerSize',20);

fig_num = 2222;
data_width = 200;
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(points_to_fit, (data_width), (fig_num));

%% Example of spiral connection between arcs
% Parameters below taken from script_test_fcn_alignArcArc
fitSequence_bestFitType{1} = 'arc';
fitSequence_bestFitType{2} = 'spiral';
fitSequence_bestFitType{3} = 'arc';

fitSequence_parameters{1}  = [0    3.0000    1.0000   -2.0944   -1.0311         0    1.0000];
fitSequence_parameters{2}  = [0.5139    2.1421    0.5397    1.3217    1.0000    1.6667];
fitSequence_parameters{3}  = [0.3281    2.8683    0.6000    0.7311    1.5708         0    1.0000];


segment_length = 0.02;
points_to_fit = fcn_geometry_plotFitSequences(fitSequence_bestFitType, fitSequence_parameters,segment_length, [], (9999));

% Add a very tiny bit of noise
points_to_fit = points_to_fit + .001*randn(size(points_to_fit));

% Analyze results, 100 times bigger
fig_num = 3333;
data_width = 200;
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(100*points_to_fit, (data_width), (fig_num));



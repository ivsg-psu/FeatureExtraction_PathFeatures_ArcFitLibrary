
%% Introduction to and Purpose of the Code
% This is the explanation of the code that can be found by running
%       script_demo_arcFit.m
% This is a script to demonstrate the functions within the arcFit code
% library. This code repo is typically located at:
%
%   https://github.com/ivsg-psu/FeatureExtraction_PathFeatures_ArcFitLibrary
%
% If you have questions or comments, please contact Sean Brennan at
% sbrennan@psu.edu
%
% The purpose of the code is to convert a set of noisy point data,
% typically measured from a mapping vehicle, into geometric constructs such
% as arcs, lines, and spirals that connect between these. 
% 
% Additional contributers include:
% 2024 - Aneesh Batchu


%% Revision History:
% 2024_08_06 - sbrennan@psu.edu
% -- started the repo by pulling codes out of Geometry class library,
% specifically from script_test_fcn_geometry_curvatureAlongCurve


%% To-do items
% 2024_08_06 - S. Brennan
% -- clear out Geometry library of:
% script_test_fcn_geometry_curvatureAlongCurve
% fcn_geometry_curvatureAlongCurve



%% Prep the workspace
close all
clc

%% Dependencies and Setup of the Code
% The code requires several other libraries to work, namely the following
%
% * DebugTools - the repo can be found at: https://github.com/ivsg-psu/Errata_Tutorials_DebugTools


% List what libraries we need, and where to find the codes for each
clear library_name library_folders library_url

ith_library = 1;
library_name{ith_library}    = 'DebugTools_v2023_04_22';
library_folders{ith_library} = {'Functions','Data'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/archive/refs/tags/DebugTools_v2023_04_22.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'PathClass_v2024_03_14';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary/archive/refs/tags/PathClass_v2024_03_14.zip';

ith_library = ith_library+1;
library_name{ith_library}    = 'GPSClass_v2023_06_29';
library_folders{ith_library} = {'Functions'};
library_url{ith_library}     = 'https://github.com/ivsg-psu/FieldDataCollection_GPSRelatedCodes_GPSClass/archive/refs/tags/GPSClass_v2023_06_29.zip';

% ith_library = ith_library+1;
% library_name{ith_library}    = 'LineFitting_v2023_07_24';
% library_folders{ith_library} = {'Functions'};
% library_url{ith_library}     = 'https://github.com/ivsg-psu/FeatureExtraction_Association_LineFitting/archive/refs/tags/LineFitting_v2023_07_24.zip';
% 
% ith_library = ith_library+1;
% library_name{ith_library}    = 'FindCircleRadius_v2023_08_02';
% library_folders{ith_library} = {'Functions'};                                
% library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_GeomTools_FindCircleRadius/archive/refs/tags/FindCircleRadius_v2023_08_02.zip';
% 
% ith_library = ith_library+1;
% library_name{ith_library}    = 'BreakDataIntoLaps_v2023_08_25';
% library_folders{ith_library} = {'Functions'};                                
% library_url{ith_library}     = 'https://github.com/ivsg-psu/FeatureExtraction_DataClean_BreakDataIntoLaps/archive/refs/tags/BreakDataIntoLaps_v2023_08_25.zip';
% 
% ith_library = ith_library+1;
% library_name{ith_library}    = 'ParseXODR_v2023_10_23';
% library_folders{ith_library} = {'Functions'};                                
% library_url{ith_library}     = 'https://github.com/ivsg-psu/PathPlanning_MapTools_ParseXODR/archive/refs/tags/ParseXODR_v2023_10_23.zip';


%% Clear paths and folders, if needed
if 1==0
    clear flag_arcFit_Folders_Initialized;
    fcn_INTERNAL_clearUtilitiesFromPathAndFolders;
end

%% Do we need to set up the work space?
if ~exist('flag_arcFit_Folders_Initialized','var')
    this_project_folders = {'Functions','Data','LargeData'};
    fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders);
    flag_arcFit_Folders_Initialized = 1;
end

%% Set environment flags for input checking
% These are values to set if we want to check inputs or do debugging
% setenv('MATLABFLAG_ARCFIT_FLAG_CHECK_INPUTS','1');
% setenv('MATLABFLAG_ARCFIT_FLAG_DO_DEBUG','1');
setenv('MATLABFLAG_ARCFIT_FLAG_CHECK_INPUTS','1');
setenv('MATLABFLAG_ARCFIT_FLAG_DO_DEBUG','0');

%% Fill Test Data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% http://patorjk.com/software/taag/#p=display&f=Big&t=Fill%20%20%20Test%20%20%20Data
%
%  ______ _ _ _     _______        _       _____        _
% |  ____(_) | |   |__   __|      | |     |  __ \      | |
% | |__   _| | |      | | ___  ___| |_    | |  | | __ _| |_ __ _
% |  __| | | | |      | |/ _ \/ __| __|   | |  | |/ _` | __/ _` |
% | |    | | | |      | |  __/\__ \ |_    | |__| | (_| | || (_| |
% |_|    |_|_|_|      |_|\___||___/\__|   |_____/ \__,_|\__\__,_|
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

flag_be_verbose = 1;
rng(1); % Fix the random number, for debugging


% Set 1==1 to use test track data. Set 1==0 to use artificial data.
if 1==1
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
XY_fig_num = 1;
figure(XY_fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

plot(points_to_fit(:,1),points_to_fit(:,2),'k.','MarkerSize',20);
plot(points_to_fit(1,1),points_to_fit(1,2),'g.','MarkerSize',20);
plot(points_to_fit(end,1),points_to_fit(end,2),'ro','MarkerSize',10);

%% Assess the data propertieso
% to find average spacing, number of points, etc.
Npoints = length(points_to_fit(:,1));
minimum_island_separation = 10; % Units are meters. See comments below for explanation

spatial_differences = diff(points_to_fit(:,1:2));
spatial_distances   = real(sum(spatial_differences.^2,2).^0.5);
average_spacing     = mean(spatial_distances);

% Check if the data is a loop (this changes a few steps that follow)
distance_XY_from_start = real(sum((points_to_fit(:,1:2) - points_to_fit(1,1:2)).^2,2).^0.5);
[max_distance_XY_from_start, index_maxDistance] = max(distance_XY_from_start);
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



%% Show how to use fcn_arcFit_curvatureAlongCurve to find any islands in geometric information
% Islands are where there are interconnected arcs that have no line
% segments within. Islands are useful for analysis because calculations
% done within an island are not affected by calculations in other islands,
% and so the data can be sub-grouped by island and processed island by
% island, thereby saving huge amounts of computation.
%
% We find islands by calculating if there are "gaps" in the
% data wherein the curvature of the data, within the gaps, is
% indistinguishable from a line fit. 
% 
% The arc curvature calculation is within the curvatureAtPoint function.
% The curvature calculation is VERY slow as it checks, via regression, all
% possible circles that can be created with the given point at the center
% of the data set. One can set number of data points to consider to right
% and left of the test point. Default is to use them all, but for large
% data this is VERY slow. Instead of using them all, we can specify the
% minimum distance ever expected between islands, e.g. the minimum distance
% allowed to be a "straightaway" on a road. This is a user-defined
% parameter that of course depends on the actual road. From this, we can
% calculate how many data points should be used for curvature calculations.
% Of note: data_width needs to be at least 2 or more for the curvature
% calculation to work.

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
data_width = 20;
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(points_to_fit, (data_width), (fig_num));

% Assign islands to locations where the SNR is less than 1, e.g. it's more
% likely that the data is a line than an arc. We make it 3 here to give it
% a bit of wiggle-room (some are right on edge).
% SNR_threshold = 0.2; % for 5 points
SNR_threshold = 3; % for 20 points
is_island = curvature_SNRs>SNR_threshold;

%% Show how to use fcn_arcFit_curvatureAlongCurve to fix loops
% In the test track, and in any data that forms a loop, it is common that
% the start/end of data will be on a curve. The results will be an island
% at start and at end, but with straightaways somehwere before and after
% the end. 
%
% If this is the case, they can be fixed by shifting the data to start at
% the middle of the first "lake" part after the starting island. This is
% basically "fixing" the loops so that the loop's start/end does not break
% a continuous island of arcs in half by putting the start/end point in the
% middle of an arc (as is the case at the test track).

lake_exists_in_data = any(~is_island);
if is_island(1,1) && is_island(end,1) && lake_exists_in_data
    % Need to fix the data

    island_starts = find(~is_island,1);
    % Make everything a lake up to where the lake starts. This makes
    % finding where the lake ends very trivial
    filled_data = is_island;
    filled_data(1:island_starts) = 0; 
    lake_ends = find(filled_data==1,1);
    mid_lake = round((island_starts+lake_ends)/2);

    % Shift the points so they start mid-lake
    shifted_points = [points_to_fit(mid_lake+1:end,:); points_to_fit(1:mid_lake,:)];
    
    % Recalculate the data using the shifted points
    fig_num = 2345;
    [curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(shifted_points, (data_width), (fig_num));
else
    shifted_points = points_to_fit;
end
is_island = curvature_SNRs>SNR_threshold;

% Plot the results
fit_fig_num = 13232;
figure(fit_fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

plot(shifted_points(:,1),shifted_points(:,2),'k.','MarkerSize',20);
plot(shifted_points(is_island,1),shifted_points(is_island,2),'r.','MarkerSize',20);

%% Use curvatureGroupAssignment to separate contiguous points into islands
fig_num = 67543;
island_ranges = fcn_INTERNAL_curvatureGroupAssignment(shifted_points, is_island, fig_num);

%% Use extractModelsUsingSNRs to convert each island into C2 arcs
SNR_threshold = 30;

clear arc_islands
arc_islands(length(island_ranges)) = struct;
for ith_island = 1:length(island_ranges)
    this_island_range = island_ranges{ith_island};
    this_island_points = shifted_points(this_island_range,:);

    % [arc_parameters, model_SNRs, sorted_model_fit_ID_at_each_index] = fcn_INTERNAL_extractModelsUsingSNRs(shifted_points, this_island_range, SNR_threshold);



    % Steps:
    % For each island, extractModelsFromCurvature function does the following
    % STEP1:  find full curvatures and SNRs
    % STEP2:  use the curvature SNRs to extract models at each island, recording model at each index
    % STEP3:  order the models in each island to be in correct order
    % STEP4:  ensure arcs have C2 continuity

    this_island_points = shifted_points(this_island_range,:);

    % STEP1: Calculate the full curvatures and SNRs of this data
    debug_fig_num = 2346;
    data_width = []; % Default is to use all possible points
    [curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(this_island_points, (data_width), (debug_fig_num));

    % STEP2: Use the curvature SNRs to extract models
    fig_num = 75655;
    [best_fit_arcs, best_fit_SNRs, ~, model_fit_ID_at_each_index] = fcn_INTERNAL_curvatureArcsFromSNR(this_island_points, curvature_SNRs, SNR_threshold, arc_centers, curvatures, point_curvature_minimum, index_ranges, fig_num);

    % STEP3: order the models to be in correct order so that 1st model is
    % the first one encountered, 2nd model is 2nd, etc.
    fig_num = 383834;
    all_segments = [];
    [~, ~, model_SNRs, sorted_model_fit_ID_at_each_index, arc_matrix, ~] = fcn_INTERNAL_curvatureModelSort(best_fit_arcs, all_segments, best_fit_SNRs, model_fit_ID_at_each_index, fig_num);

    %% URHERE
    % STEP4: ensure arcs have C2 continuity
    fig_num = 75633;
    arc_parameters = fcn_INTERNAL_alignArcsBySNRandC2(arc_matrix, model_SNRs, sorted_model_fit_ID_at_each_index, shifted_points, fig_num);


%%
    % Save results of the island calculation
    arc_islands(ith_island).this_island_range = this_island_range;
    arc_islands(ith_island).this_island_points = this_island_points;
    arc_islands(ith_island).arc_parameters = arc_parameters;
    arc_islands(ith_island).model_fit_ID_at_each_index = sorted_model_fit_ID_at_each_index;
    arc_islands(ith_island).model_SNRs = model_SNRs;
   
end
%% Save the different island results into one matrix

% Initialize model fits
entire_model_fit_ID_at_each_index = nan*shifted_points(:,1);
entire_arc_matrix_C2 = [];
entire_model_SNRs = [];
Nmodels = 0;

for ith_island = 1:length(island_ranges)
    
    % Update the models info
    this_model_IDs = arc_islands(ith_island).model_fit_ID_at_each_index;
    this_island_range = arc_islands(ith_island).this_island_range;
    this_arc_parameters = arc_islands(ith_island).arc_parameters;
    this_model_SNRs = arc_islands(ith_island).model_SNRs;

    % Set any unfilled (zero) values to NaN
    this_model_IDs(this_model_IDs==0) = nan;

    % Offset all the model IDs that were just measured so they match the
    % rows of the updated parameter list
    offset_model_fit_ID_at_each_index = Nmodels + this_model_IDs;

    % Copy these data into the correct range area.
    entire_model_fit_ID_at_each_index(this_island_range) = offset_model_fit_ID_at_each_index;

    % Update the parameter lists 
    entire_arc_matrix_C2 = [entire_arc_matrix_C2; this_arc_parameters]; %#ok<AGROW>

    % Update the count of the number of models, for the next round of the
    % loop
    if ~isempty(entire_arc_matrix_C2)
        Nmodels = length(entire_arc_matrix_C2(:,1));
    end

    % Update the SNRs
    entire_model_SNRs = [entire_model_SNRs; model_SNRs]; %#ok<AGROW>

end


%% Connect the islands with C2 line segments

NumFitsGood = length(entire_arc_matrix_C2(:,1));
revised_entire_model_fit_ID_at_each_index = entire_model_fit_ID_at_each_index;
indicies_unfilled = isnan(entire_model_fit_ID_at_each_index);

% Initialize the segments
all_segments = [];

flag_need_to_fill_start = 0;

Nsegments = 0;
while any(indicies_unfilled)

    start_index = find(indicies_unfilled==1,1);
    remainder   = indicies_unfilled;
    remainder(1:start_index) = 1; % Fill in any zeros beforehand
    end_index   = find(remainder==0,1); % Find first zero afterwards



    % Check for situation where a loop is causing the first and last areas
    % to be segments
    flag_skip_fit = 0;
    if 1==start_index && flag_is_a_loop
        % Grab the arc at end, and use it as the start
        start_index = find(~indicies_unfilled==1,1,'last');
        fit_before = entire_model_fit_ID_at_each_index(start_index);
        fit_after  = entire_model_fit_ID_at_each_index(end_index);

        % Shut off the indicies searched thus far, in prep for next round,
        % including the wrap-around portion
        indicies_unfilled(1:end_index) = 0;
        indicies_unfilled(start_index:end) = 0;

    elseif 1==start_index
        % Shut off the indicies searched thus far, in prep for next round
        indicies_unfilled(1:end_index) = 0;
        flag_skip_fit = 1;
    else
        % Grab the arc before and after the open area
        fit_before = entire_model_fit_ID_at_each_index(start_index-1);
        fit_after  = entire_model_fit_ID_at_each_index(end_index);

        % Shut off the indicies searched thus far, in prep for next round
        indicies_unfilled(1:end_index) = 0;

    end

    % Check for errors
    if isempty(end_index)
        error('Empty end indicies should never occur')
    end

    if 0==flag_skip_fit
        Nsegments = Nsegments+1;

        % Find which points should be changed. Save these indicies
        if start_index>end_index && flag_is_a_loop
            indicies_of_segment_points = [(start_index+1:Npoints)'; (1:end_index-1)'];
        else
            indicies_of_segment_points = (start_index:(end_index-1))';
        end


        arc_before = entire_arc_matrix_C2(fit_before,:);
        arc_after  = entire_arc_matrix_C2(fit_after,:);
        segment_points_to_fit = shifted_points(indicies_of_segment_points,1:2);

        if isempty(segment_points_to_fit)
            %%%
            % If segment_points_to_fit is empty?
            %
            % Fit the connecting line segment by joining arc1 to arc2 using
            % a C1 continuity. This produces a line segment exactly tangent
            % to both arcs. However, this approach ignores any points
            % between the arcs, and thus can introduce errors.
            continuity_level = 1;
            transverse_tolerance = 0;
            [revised_arc1_parameters, revised_arc2_parameters, revised_intermediate_geometry_join_type, revised_intermediate_geometry_join_parameters]  = ...
                fcn_geometry_alignArcArc(arc_before, arc_after, (transverse_tolerance), (continuity_level),  (57894));

            % Make sure a segment is produced
            assert(strcmp(revised_intermediate_geometry_join_type,'segment'));

            % Save arc revisions
            entire_arc_matrix_C2(fit_before,:) = revised_arc1_parameters;
            entire_arc_matrix_C2(fit_after,:) = revised_arc2_parameters;
            segment_parameters = revised_intermediate_geometry_join_parameters;

        else
            %%%
            % If segment_points_to_fit is NOT empty?
            %
            % Fit the connecting line segment by joining arc1 to arc2 using
            % a C1 continuity. This produces a line segment exactly tangent
            % to both arcs. However, this approach ignores any points
            % between the arcs, and thus can introduce errors.

            % The issue with this is that the arcs may not "yet" be tangent
            % to the points. To eliminate this issue, we first find the C1
            % segment, and then prune ONLY the points that are within the
            % correct area.
            continuity_level = 1;
            transverse_tolerance = 0;
            [revised_arc1_parameters, revised_arc2_parameters, revised_intermediate_geometry_join_type, revised_intermediate_geometry_join_parameters]  = ...
                fcn_geometry_alignArcArc(arc_before, arc_after, (transverse_tolerance), (continuity_level),  (57894));
            reference_segment = revised_intermediate_geometry_join_parameters;
            reference_segment_base_point = reference_segment(1,1:2);
            reference_segment_unit_tangent_vector = [cos(reference_segment(3)) sin(reference_segment(3))];
            reference_segment_length = reference_segment(1,4);

            % Find the lengths of segment_points_to_fit along the reference
            % segment
            distances_along_reference = sum((segment_points_to_fit - reference_segment_base_point).*reference_segment_unit_tangent_vector,2);
            good_indicies = (distances_along_reference>=0).*(distances_along_reference<=reference_segment_length);
            between_segment_points_to_fit = segment_points_to_fit(find(good_indicies),1:2); %#ok<FNDSB>
            
            fig_num = 57347;
            [segment_parameters, std_dev_orthogonal_distance] = fcn_geometry_fitSegmentViaRegression(between_segment_points_to_fit, fig_num);
        end

        % Move the line segment, if necessary
        fig_num = 45647;
        offset_nudge = 0.05;
        revised_segment = fcn_INTERNAL_nudgeArcArcC1SegmentToC2(segment_parameters, arc_before, arc_after, offset_nudge, fig_num);

        % Save results
        all_segments = [all_segments; revised_segment]; %#ok<AGROW>

        % Indicate where the line segments (negative numbering) are:
        revised_entire_model_fit_ID_at_each_index(indicies_of_segment_points) = -1*Nsegments;

    end
end

% Plot the revised results
arc_line_fig_num = 23434;
figure(arc_line_fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

for ith_fit = 1:length(entire_arc_matrix_C2(:,1))
    fcn_geometry_plotGeometry('arc',entire_arc_matrix_C2(ith_fit,:));
end


for ith_fit = 1:length(all_segments(:,1))
    fcn_geometry_plotGeometry('segment',all_segments(ith_fit,:));
end

%% Sort the fits to be in the correct order
fig_num = 1111;
[fitSequence_fitTypes, fitSequence_parameters, model_SNRs, sorted_model_fit_ID_at_each_index, arc_matrix, segment_matrix] = fcn_INTERNAL_curvatureModelSort(entire_arc_matrix_C2, all_segments, [], revised_entire_model_fit_ID_at_each_index, fig_num);


%% Align fits to each other to ensure C2 continuity
fig_num = 76767;
figure(fig_num); 
clf;
flag_is_a_loop = 0;
continuity_level = 2;
[revised_fitSequence_types, revised_fitSequence_parameters] = ...
    fcn_geometry_alignGeometriesInSequence(fitSequence_fitTypes, fitSequence_parameters, transverse_tolerance, (continuity_level), (flag_is_a_loop), (fig_num));

%% Analyze error of model fitting
fig_num = 2736;
figure(fig_num);
clf;

threshold           = []; %max(max_feasibility_distance,fitting_tolerance(1,2));
curve_test_segment_length = 0.05; % Check every XX meters;
[flag_is_similar, minimum_distance_to_each_point, indicies_of_nearest_reference_points, mean_error, max_error, std_dev_error] = ...
    fcn_geometry_comparePointsToCurve(...
    revised_fitSequence_types, revised_fitSequence_parameters, points_to_fit, ...
    (threshold), (curve_test_segment_length), (fig_num));





%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% fcn_INTERNAL_curvatureGroupAssignment
function island_ranges = fcn_INTERNAL_curvatureGroupAssignment(shifted_points, is_island, fig_num)
% Given an [Nx2] array of input points and an [Nx1] array flagging each
% point as an island or not, this function groups the contigous groups or
% "islands" of points into separate ranges, returning the range for each
% "island" grouping

% Find the number of islands
jumps_in_islands = [is_island(1,1); diff(is_island); 0];

% Each transition "up" from 0 to 1 will start a new island. We can count
% these to determine number of island starts. Similarly, each transition
% "down" from 1 to 0 will end the island. Again, we can count these

indicies_island_start = find(jumps_in_islands>0.5);
indicies_island_end = find(jumps_in_islands<-0.5);

% Make sure these match in length!
assert(length(indicies_island_start)==length(indicies_island_end));

% How many islands do we have in this data?
N_islands = length(indicies_island_start);

% Initialize the output
island_ranges{N_islands} = [];

% Fill the outputs
for ith_island = 1:N_islands
    island_ranges{ith_island} = (indicies_island_start(ith_island):indicies_island_end(ith_island)); 
end

% Plot the results?
if ~isempty(fig_num)
    figure(fig_num);
    clf;

    hold on;
    grid on;
    axis equal
    xlabel('X [m]');
    ylabel('Y [m]');

    % Plot the points
    plot(shifted_points(:,1),shifted_points(:,2),'k.','MarkerSize',20);

    % Plot the islands
    for ith_island = 1:N_islands
        this_island_range = island_ranges{ith_island};
        plot(shifted_points(this_island_range,1),shifted_points(this_island_range,2),'.','MarkerSize',10);
    end
end
end % Ends fcn_INTERNAL_curvatureGroupAssignment

%% fcn_INTERNAL_extractModelsUsingSNRs
function [arc_parameters, model_SNRs, sorted_model_fit_ID_at_each_index] = fcn_INTERNAL_extractModelsUsingSNRs(shifted_points, this_island_range, SNR_threshold)
% Steps:
% For each island, extractModelsFromCurvature function does the following
% STEP1:  find full curvatures and SNRs
% STEP2:  use the curvature SNRs to extract models at each island, recording model at each index
% STEP3:  order the models in each island to be in correct order
% STEP4:  ensure arcs have C2 continuity

this_island_points = shifted_points(this_island_range,:);

% STEP1: Calculate the full curvatures and SNRs of this data
debug_fig_num = 2346;
data_width = []; % Default is to use all possible points
[curvatures, arc_centers, index_ranges, point_curvature_minimum, curvature_SNRs] = fcn_arcFit_curvatureAlongCurve(this_island_points, (data_width), (debug_fig_num));

% STEP2: Use the curvature SNRs to extract models
fig_num = 75655; 
[best_fit_arcs, best_fit_SNRs, ~, model_fit_ID_at_each_index] = fcn_INTERNAL_curvatureArcsFromSNR(this_island_points, curvature_SNRs, SNR_threshold, arc_centers, curvatures, point_curvature_minimum, index_ranges, fig_num);

% STEP3: order the models to be in correct order so that 1st model is
% the first one encountered, 2nd model is 2nd, etc.
fig_num = 383834;
all_segments = [];
[~, ~, model_SNRs, sorted_model_fit_ID_at_each_index, arc_matrix, ~] = fcn_INTERNAL_curvatureModelSort(best_fit_arcs, all_segments, best_fit_SNRs, model_fit_ID_at_each_index, fig_num);

% STEP4: ensure arcs have C2 continuity
fig_num = 75633;
arc_parameters = fcn_INTERNAL_alignArcsBySNRandC2(arc_matrix, model_SNRs, sorted_model_fit_ID_at_each_index, this_island_points, fig_num);


end % Ends fcn_INTERNAL_extractModelsUsingSNRs


%% fcn_INTERNAL_curvatureArcsFromSNR
function [best_fit_arcs, best_fit_SNRs, best_fit_ranges, model_fit_ID_at_each_index] = fcn_INTERNAL_curvatureArcsFromSNR(this_island_points, curvature_SNRs, SNR_threshold, arc_centers, curvatures, point_curvature_minimum, index_ranges, fig_num)

% Find number of points
Npoints = length(this_island_points(:,1));
indiciesPoints = (1:Npoints)';

% Initialize output variables
best_fit_arcs = [];
best_fit_SNRs = [];
best_fit_ranges = [];

% Initialize internal array that keeps track of which SNRs we have already
% analyzed. If this array has an NaN value, then that point has been
% analyzed or should be ignored.
remaining_curvature_SNRs = curvature_SNRs;
remaining_curvatures = curvatures;

% Ignore any points that have an SNR below the threshold
remaining_curvature_SNRs(curvature_SNRs<SNR_threshold) = nan;
remaining_curvatures(curvature_SNRs<SNR_threshold) = nan;

% Keep track of which models go with which points by setting all the model
% numbers to zero
model_fit_ID_at_each_index = 0*curvature_SNRs;

% Set up a debugging figure
flag_do_debug = 1;
debug_fig_num = 38383;
figure(debug_fig_num);clf;

flag_first_time = 1;

NumFits = 0;

while ~all(isnan(remaining_curvature_SNRs))

    % Find the best remaining
    [~,best_SNR_index] = max(remaining_curvature_SNRs);
    NumFits = NumFits+1;
    
    % Find the index range of this fit
    min_index = best_SNR_index-index_ranges(best_SNR_index);
    max_index = best_SNR_index+index_ranges(best_SNR_index);
    
    % Make sure none are negative
    min_index = max(1,min_index);
    max_index = min(Npoints,max_index);
    this_index_range = (min_index:max_index)';

    % Plot results?
    if flag_do_debug
        %%%%%%%%%%%%%%%%%%%%%%
        figure(debug_fig_num)
        subplot(1,3,1);
        % cla;

        hold on;
        grid on;
        axis equal
        xlabel('X [m]');
        ylabel('Y [m]');

        % Make axis slightly larger?
        if 1==flag_first_time

            % Plot the input points
            plot(this_island_points(:,1),this_island_points(:,2),'b.','MarkerSize',20);


            temp = axis;
            axis_range_x = temp(2)-temp(1);
            axis_range_y = temp(4)-temp(3);
            percent_larger = 0.3;
            axis([temp(1)-percent_larger*axis_range_x, temp(2)+percent_larger*axis_range_x,  temp(3)-percent_larger*axis_range_y, temp(4)+percent_larger*axis_range_y]);
            temp_axis = axis;
            flag_first_time = 0;
        else
            axis(temp_axis);
        end

        % Plot the circle fit at the point
        fcn_geometry_plotCircle(arc_centers(best_SNR_index,:), 1/curvatures(best_SNR_index),'r-',debug_fig_num);


        % Plot the index range
        plot(this_island_points(this_index_range,1),this_island_points(this_index_range,2),'m.','MarkerSize',10)

        % Plot the max SNR point
        plot(this_island_points(best_SNR_index,1),this_island_points(best_SNR_index,2),'g.','MarkerSize',30)

        axis(temp_axis);

        title('Input points');

        %%%%%%%%%%%%%%%%%%%%%%
        subplot(1,3,2);
        % cla;

        semilogy(indiciesPoints,curvatures,'k-');
        hold on;
        semilogy(indiciesPoints,remaining_curvatures,'b-', 'LineWidth',2);
        semilogy(indiciesPoints,point_curvature_minimum,'-','Color',[0.6 0.6 0.6]);

        % Plot the part covered by this fit
        plot(indiciesPoints(this_index_range),curvatures(best_SNR_index)*ones(length(this_index_range)),'k-','Markersize',20,'LineWidth',3);

        grid on;
        xlabel('Index [count]');
        ylabel('Curvature [1/m]');
        title('Curvatures')

        %%%%%%%%%%%%%%%%%%%%%%
        subplot(1,3,3);
        % cla;
        grid on;
        hold on;

        plot(indiciesPoints, curvature_SNRs,'k-');
        plot(indiciesPoints, remaining_curvature_SNRs,'b-', 'LineWidth',2);

        % Plot the part covered by this fit
        plot(indiciesPoints(this_index_range), remaining_curvature_SNRs(this_index_range,:),'k-','LineWidth',3);

        % Plot the best point
        plot(indiciesPoints(best_SNR_index), remaining_curvature_SNRs(best_SNR_index),'g.','Markersize',30);

        xlabel('Index [count]');
        ylabel('SNR [unitless]');
        title('Curvature SNR')
    end
    
    % Save results
    best_fit_SNRs    = [best_fit_SNRs; remaining_curvature_SNRs(best_SNR_index)]; %#ok<AGROW>
    best_fit_ranges = [best_fit_ranges; index_ranges(best_SNR_index)]; %#ok<AGROW>
    

    % Block out the indicies of this fit
    remaining_curvature_SNRs(this_index_range) = nan;

    % Check which portion of these points has not yet been filled (model is
    % 0) and ONLY fill these with the current model fit number
    is_zeros = model_fit_ID_at_each_index(this_index_range)==0;
    unfilled_index_range = this_index_range(is_zeros);
    model_fit_ID_at_each_index(unfilled_index_range) = NumFits;

    % Save fit parameters. 
    % The standard arc parameter format:
    %               [circleCenter_x.
    %                circleCenter_y,
    %                radius,
    %                start_angle_in_radians,
    %                end_angle_in_radians,
    %                flag_this_is_a_circle
    %                flag_arc_is_counterclockwise
    %               ]
    best_arc_center                    = arc_centers(best_SNR_index,:);
    best_arc_radius                    = 1/curvatures(best_SNR_index);
    vector_from_circle_center_to_start = this_island_points(min_index,:)-best_arc_center;
    vector_from_circle_center_to_end   = this_island_points(max_index,:)-best_arc_center;
    best_arc_start_angle_in_radians    = mod(atan2(vector_from_circle_center_to_start(2),vector_from_circle_center_to_start(1)),2*pi);
    best_arc_end_angle_in_radians      = mod(atan2(vector_from_circle_center_to_end(2),vector_from_circle_center_to_end(1)),2*pi);
    is_counterClockwise = fcn_geometry_arcDirectionFrom3Points(this_island_points(min_index,:), this_island_points(best_SNR_index,:), this_island_points(max_index,:),-1);
    best_arc_flag_is_counterclockwise  = (1==is_counterClockwise);
    best_arc_flag_this_is_a_circle     = 0;

    best_fit_arc = [...
        best_arc_center,...
        best_arc_radius,...
        best_arc_start_angle_in_radians,...
        best_arc_end_angle_in_radians,...
        best_arc_flag_this_is_a_circle,...
        best_arc_flag_is_counterclockwise,...
        ];

    % Plot the arcs?
    if flag_do_debug
        figure(debug_fig_num)
        subplot(1,3,1);
        fcn_geometry_plotGeometry('arc',best_fit_arc);
    end

    % Save results
    best_fit_arcs = [best_fit_arcs; best_fit_arc]; %#ok<AGROW>

end

% Plot final results?
if ~isempty(fig_num)
    figure(fig_num);
    clf;

    hold on;
    grid on;
    axis equal
    xlabel('X [m]');
    ylabel('Y [m]');

    % Plot the results
    Narcs = length(best_fit_arcs(:,1));
    for ith_arc = 1:Narcs
        color_vector = fcn_geometry_fillColorFromNumberOrName(ith_arc);
        line_width = 2*Narcs - 2*ith_arc + 3;
        format_string = sprintf(' ''-'',''Color'',[%.2f %.2f %.2f],''LineWidth'',%.0d ', color_vector(1,1), color_vector(1,2), color_vector(1,3), line_width);
        fcn_geometry_plotGeometry('arc',best_fit_arcs(ith_arc,:),[],format_string);
    end

    % Plot the input points
    plot(this_island_points(:,1),this_island_points(:,2),'b.','MarkerSize',5);

    % Set the axis range
    temp = axis;
    axis_range_x = temp(2)-temp(1);
    axis_range_y = temp(4)-temp(3);
    percent_larger = 0.3;
    axis([temp(1)-percent_larger*axis_range_x, temp(2)+percent_larger*axis_range_x,  temp(3)-percent_larger*axis_range_y, temp(4)+percent_larger*axis_range_y]);

end
end % Ends fcn_INTERNAL_curvatureArcsFromSNR


%% fcn_INTERNAL_curvatureModelSort
function [fitSequence_fitTypes, fitSequence_parameters, model_SNRs, sorted_model_fit_ID_at_each_index, arc_matrix, segment_matrix] = fcn_INTERNAL_curvatureModelSort(revised_good_arcs, all_segments, best_fit_SNRs, model_fit_ID_at_each_index, fig_num)

% How many points do we have?
Npoints = length(model_fit_ID_at_each_index(:,1));

% Initialize output cell arrays and matricies
fitSequence_parameters = {};
fitSequence_fitTypes   = {};
arc_matrix = [];
segment_matrix = [];
NumFits = 0;

% Initialize the variable storing the model ordering
model_ordering = [];

% Find the first model
index_of_first_model = find(model_fit_ID_at_each_index~=0,1);

current_fit_type = 0;
for ith_point = index_of_first_model:Npoints
    this_fit = model_fit_ID_at_each_index(ith_point);
    if 0~=this_fit
        if this_fit~=current_fit_type
            NumFits = NumFits+1;
            current_fit_type = this_fit;
            model_ordering = [model_ordering; this_fit]; %#ok<AGROW>
            if current_fit_type>0
                fitSequence_fitTypes{NumFits}   = 'arc'; %#ok<AGROW>
                fitSequence_parameters{NumFits} = revised_good_arcs(current_fit_type,:); %#ok<AGROW>
                arc_matrix = [arc_matrix; revised_good_arcs(current_fit_type,:)]; %#ok<AGROW>
            else
                fitSequence_fitTypes{NumFits}   = 'segment'; %#ok<AGROW>
                fitSequence_parameters{NumFits} = all_segments(current_fit_type*-1,:); %#ok<AGROW>
                segment_matrix = [segment_matrix; all_segments(current_fit_type*-1,:)]; %#ok<AGROW>
            end
        end
    end
end

% Fix the model ordering and assign SNRs
sorted_model_fit_ID_at_each_index = 0*model_fit_ID_at_each_index;
for ith_model = 1:length(model_ordering)
    original_model_number = model_ordering(ith_model);
    indicies_to_update = model_fit_ID_at_each_index==original_model_number;
    sorted_model_fit_ID_at_each_index(indicies_to_update,1) = ith_model;
end

% Check if SNRs need to be updated
if ~isempty(best_fit_SNRs)
    model_SNRs = 0*best_fit_SNRs;
    for ith_model = 1:length(model_ordering)
        original_model_number = model_ordering(ith_model);
        model_SNRs(ith_model,1) = best_fit_SNRs(original_model_number,1);
    end
else
    model_SNRs = [];
end

% Plot the domain fits
if ~isempty(fig_num)
    figure(fig_num);
    clf;
    hold on;
    grid on;
    axis equal

    Nmodels = length(fitSequence_fitTypes);
    for ith_model = 1:Nmodels
        color = [1 0 0]*(1 - (ith_model-1)/Nmodels);
        fcn_geometry_plotGeometry(fitSequence_fitTypes{ith_model},fitSequence_parameters{ith_model},[],color,fig_num);
    end
end

end % Ends fcn_INTERNAL_curvatureModelSort


%% fcn_INTERNAL_alignArcsBySNRC2
function arc_matrix_C2 = fcn_INTERNAL_alignArcsBySNRandC2(arc_matrix, model_SNRs, best_fit_number_at_each_index, this_island_points, fig_num)
% Given fitSequence_fitTypes that are all arcs, and the arc's
% fitSequence_parameters with a SNR for each arc fit, goes through the arcs
% in order of SNR from highest to lowest, making sure that each arc is C2
% continuous with all its adjacent arcs

Npoints = length(best_fit_number_at_each_index);

% Make sure the entries are same length
NumArcs = length(arc_matrix(:,1));

% NumArcs = length(fitSequence_fitTypes);
% assert(length(fitSequence_parameters)==NumArcs);
assert(length(model_SNRs)==NumArcs);

% % Make sure all entries are 'arc' types
% assert(all(strcmp(fitSequence_fitTypes,'arc')));
% 
% % Save all the arcs into a matrix, as it's easier to work with
% arc_matrix = zeros(NumArcs,length(fitSequence_parameters{1}(1,:)));
% for ith_arc_ranking = 1:NumArcs
%     arc_matrix(ith_arc_ranking,:) = fitSequence_parameters{ith_arc_ranking}(1,:);
% end

% Sort the arcs by SNR
[~,sort_order] = sort(model_SNRs,'descend');
[~,inverse_sort_order] = sort(sort_order);

transverse_tolerance = 2;

% Initialize the output arc parameter array
arcs_with_C2_continuity = arc_matrix;

fitSequence_fitTypes{NumArcs} = [];
fitSequence_parameters{NumArcs} = [];

for ith_arc = 1:NumArcs
    fitSequence_fitTypes{ith_arc} = 'arc';
    fitSequence_parameters{ith_arc} = arc_matrix(ith_arc,:);
end

% Plot all the arcs as grey
fcn_geometry_plotFitSequences(fitSequence_fitTypes, fitSequence_parameters,[],[0.2 0.2 0.2],(fig_num));

% Proceed through the arcs from highest SNR to lowest
for ith_arc_ranking = 1:NumArcs
    this_arc = sort_order(ith_arc_ranking);
    color_number = fcn_geometry_fillColorFromNumberOrName(ith_arc_ranking);
    fcn_geometry_plotGeometry('arc',arc_matrix(this_arc,:),[],color_number,fig_num);

    if ith_arc_ranking>1

        if this_arc>1
            adjacent_before_model = this_arc-1;
        else
            adjacent_before_model = [];
        end
        if this_arc<NumArcs
            adjacent_after_model = this_arc+1;
        else
            adjacent_after_model = [];
        end

        % Which models are adjacent to this one?
        adjacent_models = [adjacent_before_model; adjacent_after_model];
        
        % What are the rankings of the adjacent models?
        rankings_to_check = inverse_sort_order(adjacent_models);

        % Only worry about rankings that are lower than this one. Rankings
        % that are lower are ones that are "fixed", such that this model
        % must move around to accomodate these prior better-fitting models.
        rankings_to_check = rankings_to_check(rankings_to_check<ith_arc_ranking);
                
        % Take the minimum of all the adjacent rankings... this is the
        % highest-ranked adjacent model, and we want to start with that
        % one. If all adjacent rankings are Nan, this returns NaN
        [priority_ranking, index_match] = min(rankings_to_check);

        % Get the lowest ranked (e.g. best) arc that is adjacent, and use
        % this as the reference
        flag_is_feasible = 1;
        if ~isempty(priority_ranking) && ~isnan(priority_ranking)
            adjacent_model_to_prioritize = adjacent_models(index_match);
            [flag_is_feasible, ~, closest_feasible_arc2_parameters] = ...
                fcn_geometry_isC2FeasibleArcToArc(arc_matrix(adjacent_model_to_prioritize,:), arc_matrix(this_arc,:), (transverse_tolerance), (0.0001), (3456));
            arcs_with_C2_continuity(this_arc,:) = closest_feasible_arc2_parameters;
        end

        % Was it feasible?
        if 1~=flag_is_feasible
            % Method that would fix this: perform C1 continuity, then shift
            % C1 line segment to get C2 continuity. 
            error('Unable to merge arcs');
            % % Perform C1 continuity 
            % [revised_arc1_parameters, revised_arc2_parameters, revised_intermediate_geometry_join_type, revised_intermediate_geometry_join_parameters]  = ...
            %     % fcn_geometry_alignArcArc(arc1_parameters, arc2_parameters, (threshold), (continuity_level),  (fig_num))
        else
            % Optimize the feasiblity
            % Align fits to each other to ensure C2 continuity
            fig_num = 12121;
            figure(fig_num);
            clf;
            flag_is_a_loop = 0;
            continuity_level = 2;
            [revised_arc_parameters, revised_X_parameters, revised_intermediate_geometry_join_type, revised_intermediate_geometry_join_parameters] = fcn_geometry_alignArcArc(...
                arc_matrix(adjacent_model_to_prioritize,:), arc_matrix(this_arc,:), (threshold), (continuity_level), (fig_num));

        end

        % Check the other side - if there's a curve there also, need to
        % make sure it still fits. If it doesn't fit, do not move it - this
        % would break the code.
        if flag_is_feasible && length(rankings_to_check)==2

            if index_match==1
                secondary_match = 2;
            else
                secondary_match = 1;
            end
            adjacent_model_to_prioritize = adjacent_models(secondary_match);

            [flag_is_feasible, ~, closest_feasible_arc2_parameters] = ...
                fcn_geometry_isC2FeasibleArcToArc(arc_matrix(adjacent_model_to_prioritize,:), closest_feasible_arc2_parameters, transverse_tolerance, (0.0001), (3456));

        end

        if 1~=flag_is_feasible
            error('Unable to fit with C2 curvature')
            % Method to fix this: remove this arc from the list and attempt
            % to connect the two adjacent to each other with C2 and then C1
        end
    end
end

arc_matrix_C2 = arcs_with_C2_continuity; 

fitSequence_fitTypes_C2 = fitSequence_fitTypes;
fitSequence_parameters_C2 = fitSequence_parameters; 
for ith_arc_ranking = 1:NumArcs
    fitSequence_parameters_C2{ith_arc_ranking} = arcs_with_C2_continuity(ith_arc_ranking,:);
end

% Plot the revised results
figure(fig_num);
clf;

hold on;
grid on;
axis equal
xlabel('X [m]');
ylabel('Y [m]');

% Plot the input sequence:
format_string = sprintf(' ''-'',''Color'',[0.6 0.6 0.6],''LineWidth'',4 ');
fcn_geometry_plotFitSequences(fitSequence_fitTypes, fitSequence_parameters,[],format_string,(fig_num));

% Plot the output sequence
format_string = sprintf(' ''-'',''Color'',[0 0 1],''LineWidth'',2 ');
fcn_geometry_plotFitSequences(fitSequence_fitTypes_C2, fitSequence_parameters_C2,[],format_string,(fig_num));

end % Ends fcn_INTERNAL_alignArcsBySNRC2

%% fcn_INTERNAL_nudgeArcArcC1SegmentToC2
function revised_segment = fcn_INTERNAL_nudgeArcArcC1SegmentToC2(segment_parameters, arc_before_parameters, arc_after_parameters, offset_nudge, fig_num)
% This function takes a segment that is used to achieve C1 continuity
% "outward" so that the segment and arcs allow C2 continuity.
%
% The line segment joining two arcs with C1 continuity is the segment that
% is tangent to both the arcs at its endpoints. This is easily calculated
% in other functions. However, for C2 continuity, this segment must be
% moved "outward" away from each of the arcs such that there is a small
% space for a spiral to join from arc1 to the line segment, then from the
% line segment to arc2. This code calculates that "shifted" segment.
%

%%%%
% Extract key variables from the parameter sets
arc_center_before = arc_before_parameters(1,1:2);
arc_center_after  = arc_after_parameters(1,1:2);
arc_radius_before = arc_before_parameters(1,3);
arc_radius_after  = arc_after_parameters(1,3);


segment_start_XY = segment_parameters(1,1:2);
segment_angle    = segment_parameters(1,3);
segment_length   = segment_parameters(1,4);
segment_end_XY   = segment_start_XY + [cos(segment_angle) sin(segment_angle)]*segment_length;
segment_unit_orthogo_vector = [cos(segment_angle+pi/2) sin(segment_angle+pi/2)];

%%%%
% Move the start and end points "outward"

% Find vectors that define direction of arc's center relative to the
% segment
vector_from_segment_start_to_arc_center_before = arc_center_before - segment_start_XY;
vector_from_segment_start_to_arc_center_after  = arc_center_after  - segment_start_XY;
distance_segment_start_to_arc_center_before = sum(segment_unit_orthogo_vector.*vector_from_segment_start_to_arc_center_before,2);
distance_segment_start_to_arc_center_after  = sum(segment_unit_orthogo_vector.*vector_from_segment_start_to_arc_center_after,2);

% Calculate how much to move the start point
if abs(distance_segment_start_to_arc_center_before)<(arc_radius_before+offset_nudge)
    % How much distance is missing?
    magnitude_nudge_start = (arc_radius_before+offset_nudge) - abs(distance_segment_start_to_arc_center_before);

    % Find the sign of the vector from start to the center, and then go the
    % opposite direction to nudge "away" from the arc center
    total_nudge_start = -1*(sign(distance_segment_start_to_arc_center_before))*magnitude_nudge_start;
else
    total_nudge_start = 0;
end
   
% Calculate how much to move the end point
if abs(distance_segment_start_to_arc_center_after)<(arc_radius_after+offset_nudge)
    % How much distance is missing?
    magnitude_nudge_after = (arc_radius_after+offset_nudge) - abs(distance_segment_start_to_arc_center_after);

    % Find the sign of the vector from start to the center, and then go the
    % opposite direction to nudge "away" from the arc center
    total_nudge_end = -1*(sign(distance_segment_start_to_arc_center_after))*magnitude_nudge_after;
else
    total_nudge_end = 0;
end

% Whichever nudge is the largest, use that one
total_nudge = max([total_nudge_start total_nudge_end]);

% Move the segment's start and end points
revised_segment_start_XY = segment_start_XY + segment_unit_orthogo_vector*total_nudge;
revised_segment_end_XY   = segment_end_XY   + segment_unit_orthogo_vector*total_nudge;

% % OLD METHOD - use the direction defined from each arc's center to each end
% % of the segment. This is straightforward. However, it does not work if the
% % segment parameters do not align with the arcs, for example if the segment
% % "hits" the arcs even after the nudge. The assumption that the segment is
% % tangent may not always be true. The modified method (above) is more
% % robust to these strange implementations, but is more complex.
% vector_from_arc_before_center_to_segment_start = segment_start_XY - arc_before_center;
% vector_from_arc_after_center_to_segment_end    = segment_end_XY   - arc_after_center;
% unit_vector_to_move_start_point = fcn_geometry_calcUnitVector(vector_from_arc_before_center_to_segment_start);
% unit_vector_to_move_end_point    = fcn_geometry_calcUnitVector(vector_from_arc_after_center_to_segment_end);
% total_nudge_start = offset_nudge;
% total_nudge_end = offset_nudge;
% revised_segment_start_XY = segment_start_XY + unit_vector_to_move_start_point*total_nudge_start;
% revised_segment_end_XY   = segment_end_XY   + unit_vector_to_move_end_point*total_nudge_end;

% Update the segment parameters
revised_segment_vector   = revised_segment_end_XY - revised_segment_start_XY;
revised_segment_angle    = atan2(revised_segment_vector(2),revised_segment_vector(1));
revised_segment_length   = real(sum(revised_segment_vector.^2,2).^0.5);
revised_segment          = [revised_segment_start_XY revised_segment_angle revised_segment_length];

if ~isempty(fig_num)
    figure(fig_num);
    clf;

    hold on;
    grid on;
    axis equal
    xlabel('X [m]');
    ylabel('Y [m]');

    fcn_geometry_plotGeometry('arc',arc_before_parameters,[],[],fig_num);
    fcn_geometry_plotGeometry('arc',arc_after_parameters,[],[],fig_num);

    format_string = sprintf(' ''-'',''Color'',[0.6 0 0],''LineWidth'',2 ');
    fcn_geometry_plotGeometry('segment',segment_parameters,[],format_string,fig_num);

    format_string = sprintf(' ''-'',''Color'',[0 0.6 0],''LineWidth'',2 ');
    fcn_geometry_plotGeometry('segment',revised_segment,[],format_string,fig_num);

end

end % Ends fcn_INTERNAL_nudgeArcArcC1SegmentToC2





%% Functions follow
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   ______                _   _
%  |  ____|              | | (_)
%  | |__ _   _ _ __   ___| |_ _  ___  _ __  ___
%  |  __| | | | '_ \ / __| __| |/ _ \| '_ \/ __|
%  | |  | |_| | | | | (__| |_| | (_) | | | \__ \
%  |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
%
% See: https://patorjk.com/software/taag/#p=display&f=Big&t=Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%§

%% function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
function fcn_INTERNAL_clearUtilitiesFromPathAndFolders
% Clear out the variables
clear global flag* FLAG*
clear flag*
clear path

% Clear out any path directories under Utilities
path_dirs = regexp(path,'[;]','split');
utilities_dir = fullfile(pwd,filesep,'Utilities');
for ith_dir = 1:length(path_dirs)
    utility_flag = strfind(path_dirs{ith_dir},utilities_dir);
    if ~isempty(utility_flag)
        rmpath(path_dirs{ith_dir});
    end
end

% Delete the Utilities folder, to be extra clean!
if  exist(utilities_dir,'dir')
    [status,message,message_ID] = rmdir(utilities_dir,'s');
    if 0==status
        error('Unable remove directory: %s \nReason message: %s \nand message_ID: %s\n',utilities_dir, message,message_ID);
    end
end

end % Ends fcn_INTERNAL_clearUtilitiesFromPathAndFolders

%% fcn_INTERNAL_initializeUtilities
function  fcn_INTERNAL_initializeUtilities(library_name,library_folders,library_url,this_project_folders)
% Reset all flags for installs to empty
clear global FLAG*

fprintf(1,'Installing utilities necessary for code ...\n');

% Dependencies and Setup of the Code
% This code depends on several other libraries of codes that contain
% commonly used functions. We check to see if these libraries are installed
% into our "Utilities" folder, and if not, we install them and then set a
% flag to not install them again.

% Set up libraries
for ith_library = 1:length(library_name)
    dependency_name = library_name{ith_library};
    dependency_subfolders = library_folders{ith_library};
    dependency_url = library_url{ith_library};
    
    fprintf(1,'\tAdding library: %s ...',dependency_name);
    fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url);
    clear dependency_name dependency_subfolders dependency_url
    fprintf(1,'Done.\n');
end

% Set dependencies for this project specifically
fcn_DebugTools_addSubdirectoriesToPath(pwd,this_project_folders);

disp('Done setting up libraries, adding each to MATLAB path, and adding current repo folders to path.');
end % Ends fcn_INTERNAL_initializeUtilities


function fcn_INTERNAL_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url, varargin)
%% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES - MATLAB package installer from URL
%
% FCN_DEBUGTOOLS_INSTALLDEPENDENCIES installs code packages that are
% specified by a URL pointing to a zip file into a default local subfolder,
% "Utilities", under the root folder. It also adds either the package
% subfoder or any specified sub-subfolders to the MATLAB path.
%
% If the Utilities folder does not exist, it is created.
%
% If the specified code package folder and all subfolders already exist,
% the package is not installed. Otherwise, the folders are created as
% needed, and the package is installed.
%
% If one does not wish to put these codes in different directories, the
% function can be easily modified with strings specifying the
% desired install location.
%
% For path creation, if the "DebugTools" package is being installed, the
% code installs the package, then shifts temporarily into the package to
% complete the path definitions for MATLAB. If the DebugTools is not
% already installed, an error is thrown as these tools are needed for the
% path creation.
%
% Finally, the code sets a global flag to indicate that the folders are
% initialized so that, in this session, if the code is called again the
% folders will not be installed. This global flag can be overwritten by an
% optional flag input.
%
% FORMAT:
%
%      fcn_DebugTools_installDependencies(...
%           dependency_name, ...
%           dependency_subfolders, ...
%           dependency_url)
%
% INPUTS:
%
%      dependency_name: the name given to the subfolder in the Utilities
%      directory for the package install
%
%      dependency_subfolders: in addition to the package subfoder, a list
%      of any specified sub-subfolders to the MATLAB path. Leave blank to
%      add only the package subfolder to the path. See the example below.
%
%      dependency_url: the URL pointing to the code package.
%
%      (OPTIONAL INPUTS)
%      flag_force_creation: if any value other than zero, forces the
%      install to occur even if the global flag is set.
%
% OUTPUTS:
%
%      (none)
%
% DEPENDENCIES:
%
%      This code will automatically get dependent files from the internet,
%      but of course this requires an internet connection. If the
%      DebugTools are being installed, it does not require any other
%      functions. But for other packages, it uses the following from the
%      DebugTools library: fcn_DebugTools_addSubdirectoriesToPath
%
% EXAMPLES:
%
% % Define the name of subfolder to be created in "Utilities" subfolder
% dependency_name = 'DebugTools_v2023_01_18';
%
% % Define sub-subfolders that are in the code package that also need to be
% % added to the MATLAB path after install; the package install subfolder
% % is NOT added to path. OR: Leave empty ({}) to only add
% % the subfolder path without any sub-subfolder path additions.
% dependency_subfolders = {'Functions','Data'};
%
% % Define a universal resource locator (URL) pointing to the zip file to
% % install. For example, here is the zip file location to the Debugtools
% % package on GitHub:
% dependency_url = 'https://github.com/ivsg-psu/Errata_Tutorials_DebugTools/blob/main/Releases/DebugTools_v2023_01_18.zip?raw=true';
%
% % Call the function to do the install
% fcn_DebugTools_installDependencies(dependency_name, dependency_subfolders, dependency_url)
%
% This function was written on 2023_01_23 by S. Brennan
% Questions or comments? sbrennan@psu.edu

% Revision history:
% 2023_01_23:
% -- wrote the code originally
% 2023_04_20:
% -- improved error handling
% -- fixes nested installs automatically

% TO DO
% -- Add input argument checking

flag_do_debug = 0; % Flag to show the results for debugging
flag_do_plots = 0; % % Flag to plot the final results
flag_check_inputs = 1; % Flag to perform input checking

if flag_do_debug
    st = dbstack; %#ok<*UNRCH>
    fprintf(1,'STARTING function: %s, in file: %s\n',st(1).name,st(1).file);
end


%% check input arguments
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____                   _
%  |_   _|                 | |
%    | |  _ __  _ __  _   _| |_ ___
%    | | | '_ \| '_ \| | | | __/ __|
%   _| |_| | | | |_) | |_| | |_\__ \
%  |_____|_| |_| .__/ \__,_|\__|___/
%              | |
%              |_|
% See: http://patorjk.com/software/taag/#p=display&f=Big&t=Inputs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if flag_check_inputs
    % Are there the right number of inputs?
    narginchk(3,4);
end

%% Set the global variable - need this for input checking
% Create a variable name for our flag. Stylistically, global variables are
% usually all caps.
flag_varname = upper(cat(2,'flag_',dependency_name,'_Folders_Initialized'));

% Make the variable global
eval(sprintf('global %s',flag_varname));

if nargin==4
    if varargin{1}
        eval(sprintf('clear global %s',flag_varname));
    end
end

%% Main code starts here
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   __  __       _
%  |  \/  |     (_)
%  | \  / | __ _ _ _ __
%  | |\/| |/ _` | | '_ \
%  | |  | | (_| | | | | |
%  |_|  |_|\__,_|_|_| |_|
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



if ~exist(flag_varname,'var') || isempty(eval(flag_varname))
    % Save the root directory, so we can get back to it after some of the
    % operations below. We use the Print Working Directory command (pwd) to
    % do this. Note: this command is from Unix/Linux world, but is so
    % useful that MATLAB made their own!
    root_directory_name = pwd;
    
    % Does the directory "Utilities" exist?
    utilities_folder_name = fullfile(root_directory_name,'Utilities');
    if ~exist(utilities_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(root_directory_name,'Utilities');
        
        % Did it work?
        if ~success_flag
            error('Unable to make the Utilities directory. Reason: %s with message ID: %s\n',error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The Utilities directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',error_message, message_ID);
        end
        
    end
    
    % Does the directory for the dependency folder exist?
    dependency_folder_name = fullfile(root_directory_name,'Utilities',dependency_name);
    if ~exist(dependency_folder_name,'dir')
        % If we are in here, the directory does not exist. So create it
        % using mkdir
        [success_flag,error_message,message_ID] = mkdir(utilities_folder_name,dependency_name);
        
        % Did it work?
        if ~success_flag
            error('Unable to make the dependency directory: %s. Reason: %s with message ID: %s\n',dependency_name, error_message,message_ID);
        elseif ~isempty(error_message)
            warning('The %s directory was created, but with a warning: %s\n and message ID: %s\n(continuing)\n',dependency_name, error_message, message_ID);
        end
        
    end
    
    % Do the subfolders exist?
    flag_allFoldersThere = 1;
    if isempty(dependency_subfolders{1})
        flag_allFoldersThere = 0;
    else
        for ith_folder = 1:length(dependency_subfolders)
            subfolder_name = dependency_subfolders{ith_folder};
            
            % Create the entire path
            subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
            
            % Check if the folder and file exists that is typically created when
            % unzipping.
            if ~exist(subfunction_folder,'dir')
                flag_allFoldersThere = 0;
            end
        end
    end
    
    % Do we need to unzip the files?
    if flag_allFoldersThere==0
        % Files do not exist yet - try unzipping them.
        save_file_name = tempname(root_directory_name);
        zip_file_name = websave(save_file_name,dependency_url);
        % CANT GET THIS TO WORK --> unzip(zip_file_url, debugTools_folder_name);
        
        % Is the file there?
        if ~exist(zip_file_name,'file')
            error(['The zip file: %s for dependency: %s did not download correctly.\n' ...
                'This is usually because permissions are restricted on ' ...
                'the current directory. Check the code install ' ...
                '(see README.md) and try again.\n'],zip_file_name, dependency_name);
        end
        
        % Try unzipping
        unzip(zip_file_name, dependency_folder_name);
        
        % Did this work? If so, directory should not be empty
        directory_contents = dir(dependency_folder_name);
        if isempty(directory_contents)
            error(['The necessary dependency: %s has an error in install ' ...
                'where the zip file downloaded correctly, ' ...
                'but the unzip operation did not put any content ' ...
                'into the correct folder. ' ...
                'This suggests a bad zip file or permissions error ' ...
                'on the local computer.\n'],dependency_name);
        end
        
        % Check if is a nested install (for example, installing a folder
        % "Toolsets" under a folder called "Toolsets"). This can be found
        % if there's a folder whose name contains the dependency_name
        flag_is_nested_install = 0;
        for ith_entry = 1:length(directory_contents)
            if contains(directory_contents(ith_entry).name,dependency_name)
                if directory_contents(ith_entry).isdir
                    flag_is_nested_install = 1;
                    install_directory_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name);
                    install_files_from = fullfile(directory_contents(ith_entry).folder,directory_contents(ith_entry).name,'*'); % BUG FIX - For Macs, must be *, not *.*
                    install_location_to = fullfile(directory_contents(ith_entry).folder);
                end
            end
        end
        
        if flag_is_nested_install
            [status,message,message_ID] = movefile(install_files_from,install_location_to);
            if 0==status
                error(['Unable to move files from directory: %s\n ' ...
                    'To: %s \n' ...
                    'Reason message: %s\n' ...
                    'And message_ID: %s\n'],install_files_from,install_location_to, message,message_ID);
            end
            [status,message,message_ID] = rmdir(install_directory_from);
            if 0==status
                error(['Unable remove directory: %s \n' ...
                    'Reason message: %s \n' ...
                    'And message_ID: %s\n'],install_directory_from,message,message_ID);
            end
        end
        
        % Make sure the subfolders were created
        flag_allFoldersThere = 1;
        if ~isempty(dependency_subfolders{1})
            for ith_folder = 1:length(dependency_subfolders)
                subfolder_name = dependency_subfolders{ith_folder};
                
                % Create the entire path
                subfunction_folder = fullfile(root_directory_name, 'Utilities', dependency_name,subfolder_name);
                
                % Check if the folder and file exists that is typically created when
                % unzipping.
                if ~exist(subfunction_folder,'dir')
                    flag_allFoldersThere = 0;
                end
            end
        end
        % If any are not there, then throw an error
        if flag_allFoldersThere==0
            error(['The necessary dependency: %s has an error in install, ' ...
                'or error performing an unzip operation. The subfolders ' ...
                'requested by the code were not found after the unzip ' ...
                'operation. This suggests a bad zip file, or a permissions ' ...
                'error on the local computer, or that folders are ' ...
                'specified that are not present on the remote code ' ...
                'repository.\n'],dependency_name);
        else
            % Clean up the zip file
            delete(zip_file_name);
        end
        
    end
    
    
    % For path creation, if the "DebugTools" package is being installed, the
    % code installs the package, then shifts temporarily into the package to
    % complete the path definitions for MATLAB. If the DebugTools is not
    % already installed, an error is thrown as these tools are needed for the
    % path creation.
    %
    % In other words: DebugTools is a special case because folders not
    % added yet, and we use DebugTools for adding the other directories
    if strcmp(dependency_name(1:10),'DebugTools')
        debugTools_function_folder = fullfile(root_directory_name, 'Utilities', dependency_name,'Functions');
        
        % Move into the folder, run the function, and move back
        cd(debugTools_function_folder);
        fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        cd(root_directory_name);
    else
        try
            fcn_DebugTools_addSubdirectoriesToPath(dependency_folder_name,dependency_subfolders);
        catch
            error(['Package installer requires DebugTools package to be ' ...
                'installed first. Please install that before ' ...
                'installing this package']);
        end
    end
    
    
    % Finally, the code sets a global flag to indicate that the folders are
    % initialized.  Check this using a command "exist", which takes a
    % character string (the name inside the '' marks, and a type string -
    % in this case 'var') and checks if a variable ('var') exists in matlab
    % that has the same name as the string. The ~ in front of exist says to
    % do the opposite. So the following command basically means: if the
    % variable named 'flag_CodeX_Folders_Initialized' does NOT exist in the
    % workspace, run the code in the if statement. If we look at the bottom
    % of the if statement, we fill in that variable. That way, the next
    % time the code is run - assuming the if statement ran to the end -
    % this section of code will NOT be run twice.
    
    eval(sprintf('%s = 1;',flag_varname));
end


%% Plot the results (for debugging)?
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _____       _
%  |  __ \     | |
%  | |  | | ___| |__  _   _  __ _
%  | |  | |/ _ \ '_ \| | | |/ _` |
%  | |__| |  __/ |_) | |_| | (_| |
%  |_____/ \___|_.__/ \__,_|\__, |
%                            __/ |
%                           |___/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_do_plots
    
    % Nothing to do!
    
    
    
end

if flag_do_debug
    fprintf(1,'ENDING function: %s, in file: %s\n\n',st(1).name,st(1).file);
end

end % Ends function fcn_DebugTools_installDependencies

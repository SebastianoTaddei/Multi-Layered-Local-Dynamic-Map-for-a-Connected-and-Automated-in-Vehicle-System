function [out] = visualise_data(path, city, multiple_avs, full)
% Prepares data to be visualised

% Load data
loaded_data = load(path).DATA;
fields      = fieldnames(loaded_data);

% Get the common end time
end_time = 0;
for i = 1:length(fields)
    in_data   = loaded_data.(fields{i});
    in_fields = fieldnames(in_data);
    for j = 1:length(in_fields)
        if end_time == 0 || end_time > length(in_data.(in_fields{j}))
            end_time = length(in_data.(in_fields{j}));
        end
    end
end

% Get date
date_str   = extractBetween(path, 'DATA', '.mat');
in_format  = 'yyyy.MM.dd_HH.mm.ss.SSS';
out_format = 'yyyy-MM-dd HH:mm:ss.SSS';
date       = datetime(strcat(date_str{1}, '.000'), 'InputFormat', in_format, 'Format', out_format);

% Prepare struct
data             = struct();
data.VideoObject = [];
data.RO          = [];
data.RV          = [];
data.HV          = [];
data.GNSS        = [];

% Fill struct
for i = 1:length(fields)
    in_data   = loaded_data.(fields{i});
    in_fields = fieldnames(in_data);
    TT        = [];

    if contains(fields{i}, 'VideoObject')
        for j = 1:length(in_fields)
            time = date + seconds(round(in_data.(in_fields{j})(1:end_time, 1), 3));
            TT = [TT, timetable(time, in_data.(in_fields{j})(1:end_time, 2), 'VariableNames', in_fields(j))]; %#ok<*AGROW>
        end
        data.VideoObject = [data.VideoObject; TT];
    end

    if contains(fields{i}, 'RO')
        for j = 1:length(in_fields)
            time = date + seconds(round(in_data.(in_fields{j})(1:end_time, 1), 3));
            TT = [TT, timetable(time, in_data.(in_fields{j})(1:end_time, 2), 'VariableNames', in_fields(j))];
        end
        data.RO = [data.RO; TT];
    end

    if contains(fields{i}, 'RV')
        for j = 1:length(in_fields)
            time = date + seconds(round(in_data.(in_fields{j})(1:end_time, 1), 3));
            TT = [TT, timetable(time, in_data.(in_fields{j})(1:end_time, 2), 'VariableNames', in_fields(j))];
        end
        TT = [TT, timetable(time, repmat(str2double(extractAfter(fields{i}, '_')), length(time), 1), 'VariableNames', {'ID'})];
        data.RV = [data.RV; TT];
    end

    if contains(fields{i}, 'HV')
        for j = 1:length(in_fields)
            time = date + seconds(round(in_data.(in_fields{j})(1:end_time, 1), 3));
            TT = [TT, timetable(time, in_data.(in_fields{j})(1:end_time, 2), 'VariableNames', in_fields(j))];
        end
        data.HV = [data.HV; TT];
    end

    if contains(fields{i}, 'GNSS')
        for j = 1:length(in_fields)
            time = date + seconds(round(in_data.(in_fields{j})(1:end_time, 1), 3));
            TT = [TT, timetable(time, in_data.(in_fields{j})(1:end_time, 2), 'VariableNames', in_fields(j))];
        end
        data.GNSS = [data.GNSS; TT];
    end
end

if full
    % Add ellipse data to detections
    data.VideoObject = [data.VideoObject,...
        repmat(data.GNSS(:,...
        {'Smjr_ellipse_orient_n', 'Smjr_ax_ellipse', 'Smnr_ax_ellipse'}), ...
        10, 1)];
end

% Group data
data.ego         = sortrows([data.HV, data.GNSS]);
data.detections  = sortrows([data.VideoObject, data.RO]);
data.connections = sortrows(data.RV);

% Clean data
data.ego         = rmmissing(data.ego);
data.detections  = rmmissing(data.detections);
data.connections = rmmissing(data.connections);

if strcmp(city, 'Trento')
    % For Trento's data
    min_lat = 46;
    max_lat = 46.1667;
    min_lon = 11;
    max_lon = 11.1667;
elseif strcmp(city, 'Orbassano')
    % For Orbassano's data
    min_lat = 44.9667;
    max_lat = 45.0667;
    min_lon = 7.5;
    max_lon = 7.6667;
end

data.ego(...
    or(or(data.ego{:, 'Lat'} < min_lat, data.ego{:, 'Lat'} > max_lat),...
    or(data.ego{:, 'Lon'} < min_lon, data.ego{:, 'Lon'} > max_lon)), :)  = [];
data.detections(...
    or(or(data.detections.Lat < min_lat, data.detections.Lat > max_lat),...
    or(data.detections.Lon < min_lon, data.detections.Lon > max_lon)), :)  = [];
if multiple_avs
    data.connections(...
        or(or(data.connections{:, 'Lat'} < min_lat, data.connections{:, 'Lat'} > max_lat),...
        or(data.connections{:, 'Lon'} < min_lon, data.connections{:, 'Lon'} > max_lon)), :)  = [];
end

% Camera classes are:
% - 0, 4, 7 -> unknown
% - 1 -> car
% - 2 -> truck
% - 3 -> cycle
% - 5 -> pedestrian
% - 6 -> bicycle
detection_classes = {...
    'Unknown',...
    'Car',...
    'Truck',...
    'Cycle',...
    'Unknown',...
    'Pedestrian',...
    'Bicycle',...
    'Unknown'...
    };

% Connection classes are:
% - 0 -> unknown
% - 1 -> pedestrian
% - 2 -> cyclist
% - 3 -> moped
% - 4 -> motorcycle
% - 5 -> passengerCar
% - 6 -> bus
% - 7 -> lightTruck
% - 8 -> heavyTruck
% - 9 -> trailer
% - 10 -> specialVehicles
% - 11 -> tram
% - 12 -> roadSideUnit
connection_classes = {...
    'Unknown',...
    'Pedestrian',...
    'Cyclist',...
    'Moped',...
    'Motorcycle',...
    'Pedestrian',...
    'PassengerCar',...
    'Bus',...
    'LightTruck',...
    'HeavyTruck',...
    'Trailer',...
    'SpecialVehicles',...
    'Tram',...
    'RoadSideUnit'...
    };

% Label data
data.detections = [data.detections, timetable(data.detections.time,...
    repmat({'placeholder'}, height(data.detections), 1),...
    11112000.*ones(height(data.detections), 1),...
    zeros(height(data.detections), 1),...
    'VariableNames', {'type', 'refID', 'delay'})];

for i = 1:height(data.detections)
    data.detections{i, 'type'} = detection_classes(data.detections{i, 'class'} + 1);
end

% Plot data
% [~, mask_handle] = unique(data.detections.handle, 'stable');
% mask_class       = find(data.detections.class == 5);
% mask             = intersect(mask_handle, mask_class);
% geoscatter(...
%     data.detections.Lat(mask),...
%     data.detections.Lon(mask))
% text(data.detections.Lat(mask),...
%     data.detections.Lon(mask)+2e-5,...
%     datestr(data.detections.time(mask), 'HH:MM:SS.FFF'));
geoscatter(...
    data.detections.Lat,...
    data.detections.Lon)

% Output data for debugging
out = data;
end


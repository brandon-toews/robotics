function mapData = readYAMLFile(filename)
    % Read YAML file contents
    fid = fopen(filename, 'r');
    content = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    content = content{1};
    
    % Parse key fields
    mapData = struct();
    
    for i = 1:length(content)
        line = content{i};
        if contains(line, 'image:')
            mapData.image = strtrim(extractAfter(line, 'image:'));
        elseif contains(line, 'resolution:')
            mapData.resolution = str2double(strtrim(extractAfter(line, 'resolution:')));
        elseif contains(line, 'origin:')
            % Parse origin array [x, y, theta]
            originStr = extractAfter(line, 'origin:');
            originStr = strrep(originStr, '[', '');
            originStr = strrep(originStr, ']', '');
            originVals = str2double(strsplit(originStr, ','));
            mapData.origin = originVals;
        elseif contains(line, 'occupied_thresh:')
            mapData.occupied_thresh = str2double(strtrim(extractAfter(line, 'occupied_thresh:')));
        end
    end
end
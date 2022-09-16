function duplicate_camera(dannce_path)
    load(dannce_path)
    for i = 5:6
        camnames{i} = sprintf('Camera%d',i);
        labelData{i} = labelData{i-4};
        params{i} = params{i-4};
        sync{i} = sync{i-4};
    end
    save(dannce_path)
end
function merge_com_dannce(dannce_path)
    load(dannce_path)
    com_path = [fileparts(dannce_path) filesep 'com3d.mat'];
    coms = load(com_path);
    com.com3d = coms.com;
    com.sampleID = coms.sampleID;
    clear coms
    clear com_path
    
    save(dannce_path)
end
function [poseStruct, recordStruct] = preprocess_prediction(predictionFile, recordingFile, gpioFile, syncFile, skeleton)

% Inputs: predictionFile: a .mat file containing a predictions.mat DANNCE struct, with a
%                       struct containing each markers x-y-z prediction
%
%         fileoutput: a .mat file
%
%         animalname: the name of the animal (default - "rats") that
%             defines the number of links and markers to use for a given
%             dataset in load_link_files.m. Other pre-defined options
%             include: 'mouse' (14 marker) kyle_mouse (20 marker)
%
%         input_params: a struct containing experiment specific
%                       information: the markers to use as
%                       SpineF (SpineF_marker) and SpineM (SpineM_marker)
%                       to align the animal, the relative framerate to 300
%                       Hz (repfactor = 300/experiment framerate),
%                       conversion factor (to scale the outputs)

%% Load and Process varibles
predResult = load(predictionFile);

f = fieldnames(predResult.predictions);
v = struct2cell(predResult.predictions);
a = cell2struct(v,f);

predStruct.predictions = a;

if isfield(predStruct.predictions,'sampleID')
    predStruct.sampleID = predStruct.predictions.sampleID;
    predStruct.predictions =rmfield(predStruct.predictions,'sampleID');
end

% Rename tails and remove tail_end_
skeleton.joint_names{6} = 'Tail_base_';
skeleton.joint_names{7} = 'Tail_mid_';
skeleton.joint_names(8) = [];
skeleton.joints_idx(10,:) = [];
skeleton.color(10,:) = [];
markercolors  = {'r','r','r',... % Head
                 'g','g',... % Spine
                 'g','g',... % Tail
                 'b','b','b','b', ... % L arm
                 'c','c','c','c', ... % R arm
                 'm','m','m',... % L leg
                 'y','y','y',... % R leg
                 };

predStruct.joints = skeleton.joint_names;
predStruct.links = num2cell(skeleton.joints_idx,2);
predStruct.linkcolor = num2cell(skeleton.color,2);
predStruct.markercolor = markercolors;
predStruct.predictions = rmfield(predStruct.predictions,'Tail_end_');

%% Preprocessing setup
markernames = predStruct.joints;

% Gaussian + moving median filter
for idx = 1:numel(markernames)
    predStruct.predictions.(markernames{idx}) = smoothdata(predStruct.predictions.(markernames{idx}), 'movmedian', 3);
end

% Get Features
poseStruct = compute_pose_feature(predStruct);

%% Process recording 
gpio = readtable(gpioFile);
gpio = gpio(strcmp(gpio.ChannelName, 'GPIO-1'), [1 3]);
gpio = table2array(gpio);

firstBlink = gpio(find(gpio(:,2) > 25000, 1), 1);

recordStruct = load(recordingFile);
recordRate = recordStruct.neuron.frame_range(2) / max(gpio(:,1));
recordingTime = round((1:recordStruct.neuron.frame_range(2)) / recordRate * 1000);

load(syncFile)
poseStruct.alignedTime = poseStruct.sampleID - (poseStruct.sampleID(startFrame) - firstBlink * 1000);
[~, ~, recordingMatch] = histcounts(poseStruct.alignedTime, recordingTime);

recordStruct.trace = recordStruct.neuron.C(:,recordingMatch);
recordStruct.traceRaw = recordStruct.neuron.C_raw(:,recordingMatch);

%% Check time-synchronization
runCheck = false;

if runCheck
    onTime = gpio(find(gpio(:,2) > 25000), 1) * 1000;
    [~, ~, checkTime] = histcounts(onTime, predStruct.alignedTime);
    checkTime = unique(checkTime(checkTime >0));
    
    vidPaths = '/Users/hyungju/Desktop/hyungju/Result/functional-stn/220308-wt22/videos/Camera2/0.mp4';
    new_frame = [];
    video_temp = VideoReader(vidPaths);
    frameCount = 0;
    videoIdx = 0;
    
    for frame = 1:50000
        frameCount = frameCount + 1;
        if frameCount > 10000
            frameCount = frameCount - 10000;
            videoIdx = videoIdx + 10000;
            video_temp = VideoReader([vidPaths(1:end-5) num2str(videoIdx) '.mp4']);
        end
        temp = readFrame(video_temp);
        if frame < 40000
            continue
        end
        if ismember(frame, checkTime)
            sprintf('Checking Frame %d in %d / %d in %d.mp4', frame, predStruct.alignedTime(frame), video_temp.NumFrames, videoIdx)
            image(temp)
            drawnow()
        end
    end
end



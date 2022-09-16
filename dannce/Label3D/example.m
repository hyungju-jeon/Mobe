%% Example setup for Label3D
% Label3D is a GUI for manual labeling of 3D keypoints in multiple cameras.
%
% Its main features include:
% 1. Simultaneous viewing of any number of camera views.
% 2. Multiview triangulation of 3D keypoints.
% 3. Point-and-click and draggable gestures to label keypoints.
% 4. Zooming, panning, and other default Matlab gestures
% 5. Integration with Animator classes.
% 6. Support for editing prelabeled data.
%
% Instructions:
% right: move forward one frameRate
% left: move backward one frameRate
% up: increase the frameRate
% down: decrease the frameRate
% t: triangulate points in current frame that have been labeled in at least two images and reproject into each image
% r: reset gui to the first frame and remove Animator restrictions
% u: reset the current frame to the initial marker positions
% z: Toggle zoom state
% p: Show 3d animation plot of the triangulated points.
% backspace: reset currently held node (first click and hold, then
%            backspace to delete)
% pageup: Set the selectedNode to the first node
% tab: shift the selected node by 1
% shift+tab: shift the selected node by -1
% h: print help messages for all Animators
% shift+s: Save the data to a .mat file
clear all
close all;
addpath(genpath('deps'))
addpath(genpath('skeletons'))
% danncePath = 'Y:/Diego/code/DANNCE';

%% Load in the calibration parameter data
% projectFolder = fullfile('/Users/hyungju/Desktop/hyungju/Result/functional-stn/220308-wt22/');
% projectFolder = fullfile('/Users/hyungju/Desktop/hyungju/Result/functional-stn/220406-pv24/');
projectFolder = fullfile('/Users/hyungju/Desktop/hyungju/Result/functional-stn/220415-pv24/');
% projectFolder = fullfile('/Users/hyungju/Desktop/hyungju/Result/functional-stn/220104-reference/');

load(fullfile([projectFolder 'calibration' filesep 'extrinsic' filesep 'camera_params.mat']))

for camerause = 1:4
    RDistort = params_individual{camerause}.RadialDistortion;
    TDistort = params_individual{camerause}.TangentialDistortion;
    r = rotationMatrix{camerause};
    t = translationVector{camerause};
    K = params_individual{camerause}.IntrinsicMatrix;
    param_name = [projectFolder 'calibration' filesep 'hires_cam' num2str(camerause) '_params.mat'];
    if ~exist(param_name, 'file')
        save([projectFolder 'calibration' filesep 'hires_cam' num2str(camerause) '_params.mat'],'K','r','t','RDistort','TDistort')
    end
end

calibPaths = collectCalibrationPaths(projectFolder);
params = cellfun(@(X) {load(X)}, calibPaths);

pause

%% Load the videos into memory
vidName = '0.mp4';
vidPaths = collectVideoPaths(projectFolder,vidName);
videos = cell(4,1);
sync = collectSyncPaths(projectFolder, '*.mat');
sync = cellfun(@(X) {load(X)}, sync);

%%
% In case the demo folder uses the dannce.mat data format.
framesToLabel = 1:100:40000;
delete(gcp('nocreate'))

video_temp = VideoReader(vidPaths{3});
temp = readFrame(video_temp);
image(temp)
[x,y] = ginput(1);
gray_val = impixel(temp,x,y);
close all

poolObj = parpool(4);
parfor nVid = 1:numel(vidPaths)
    new_frame = [];
    sprintf('Processing video %d / %d', nVid,numel(vidPaths)) 
    video_temp = VideoReader(vidPaths{nVid});
    frameCount = 0;
    videoIdx = 0;
    for frame = 1:max(framesToLabel)
        frameCount = frameCount + 1;
        if frameCount > 10000
            frameCount = frameCount - 10000;
            videoIdx = videoIdx + 10000;
            video_temp = VideoReader([vidPaths{nVid}(1:end-5) num2str(videoIdx) '.mp4']);
        end
        if mod(frame,100)==0
            sprintf('Processing frame %d in %d / %d in %d.mp4', frame, frameCount, video_temp.NumFrames, videoIdx)
        end
        temp = readFrame(video_temp);
        if ismember(frame, framesToLabel)
            temp = chromadapt(temp, gray_val);
            if isempty(new_frame)
                new_frame = temp;
            else
                new_frame = cat(4,new_frame, temp);
            end
        end
    end
%     for count = 1:nFrame
%         if count == 1
%             new_frame = readFrame(video_temp);
%             new_frame = imsharpen(new_frame(:,:,[3, 2, 1]), 'Amount', 5, 'Radius',2);
%         else
%             temp = readFrame(video_temp);
%             new_frame = cat(4,new_frame, imsharpen(temp(:,:,[3, 2, 1]), 'Amount', 5, 'Radius',2));
%         end
%     end
    videos{nVid} = new_frame;
end
delete(gcp('nocreate'))

%% Get the skeleton
skeleton = load('skeletons/mouse22');
% skeleton = load('com'); 

%% Calibrate small movements
run_calibrate = false;
dannceFolder = [projectFolder 'dannce'];
mkdir(dannceFolder)
close all

if run_calibrate
    labelGui = Calibrate3D(params, videos, skeleton, 'sync', sync, 'framesToLabel', framesToLabel, 'savePath', dannceFolder, 'Calibrate', 1);
    pause
    params = labelGui.origCamParams;
    
    for camerause = 1:4
        RDistort = params{camerause}.RDistort;
        TDistort = params{camerause}.TDistort;
        r = params{camerause}.r;
        t = params{camerause}.t;
        K = params{camerause}.K;
        param_name = [projectFolder 'calibration' filesep 'hires_cam' num2str(camerause) '_params.mat'];
        save([projectFolder 'calibration' filesep 'hires_cam' num2str(camerause) '_params.mat'],'K','r','t','RDistort','TDistort')
    end
end

%% Start Label3D
run_train = true;
close all
dannce_history = rdir([dannceFolder filesep '*_Label3D.mat']);
if run_train
    if isempty(dannce_history)
        labelGui = Label3D(params, videos, skeleton, 'sync', sync, 'framesToLabel', framesToLabel, 'savePath', dannceFolder);
    else
        latest_file = dannce_history(end).name;
        labelGui = Label3D(latest_file, videos, 'sync', sync, 'framesToLabel', framesToLabel, 'savePath', dannceFolder);
    end
    pause
end

%% If you just wish to view labels, use View 3D
% close all
% viewGui = View3D(params, videos, skeleton);

%% Check prediction
videos_all = cell(4,1);

framesToTest = 1:10:10000;
delete(gcp('nocreate'))
poolObj = parpool(4);
parfor nVid = 1:numel(vidPaths)
    new_frame = [];
    sprintf('Processing video %d / %d', nVid,numel(vidPaths))
    video_temp = VideoReader(vidPaths{nVid});
    frameCount = 0;
    videoIdx = 0;
    for frame = 1:max(framesToTest)
        frameCount = frameCount + 1;
        if frameCount > 10000
            frameCount = frameCount - 10000;
            videoIdx = videoIdx + 10000;
            video_temp = VideoReader([vidPaths{nVid}(1:end-5) num2str(videoIdx) '.mp4']);
        end
        if mod(frame,10)==0
            sprintf('Processing frame %d in %d / %d in %d.mp4', frame, frameCount, video_temp.NumFrames, videoIdx)
        end
        temp = readFrame(video_temp);
        if ismember(frame, framesToTest)
            if isempty(new_frame)
                new_frame = temp;
            else
                new_frame = cat(4,new_frame, temp);
            end
        end
        
    end
    videos_all{nVid} = new_frame;
end
delete(gcp('nocreate'))

%% Show COM prediction
com = load([projectFolder 'dannce/com3d.mat']);
com2 = repmat(com.com(framesToTest,:), 1, 1, 22);

close all
labelGui = Label3D(params, videos_all, skeleton, 'sync', sync, 'framesToLabel', framesToTest);
labelGui.loadFrom3D(com2)

%% Show Prediction
pred = load([projectFolder 'dannce/predictions.mat']);
pred.predictions = rmfield(pred.predictions, 'sampleID');

temp = zeros(22, 3, length(framesToTest));
for i = 1:length(framesToTest)
    A = cell2mat(structfun(@(X) {X(framesToTest(i),:)}, pred.predictions));
    temp(:,:,i) = A;
end

close all
labelGui = Label3D(params, videos_all, skeleton, 'sync', sync, 'framesToLabel', framesToTest);
labelGui.loadFrom3D(permute(temp, [3,2,1]))

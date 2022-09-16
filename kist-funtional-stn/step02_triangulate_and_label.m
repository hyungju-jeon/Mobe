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
addpath('/Users/hyungju/Desktop/hyungju/Code/python/dannce/Label3D/deps')
addpath('/Users/hyungju/Desktop/hyungju/Code/python/dannce/Label3D/skeletons')
addpath('/Users/hyungju/Desktop/hyungju/Code/python/dannce/Label3D')

%% Load in the calibration parameter data
projectFolder = fullfile('/Users/hyungju/Desktop/hyungju/Result/functional-stn/220831-takeOver/');

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

%% Load the videos into memory
vidName = '0.mp4';
vidPaths = collectVideoPaths(projectFolder,vidName);
videos = cell(4,1);
sync = collectSyncPaths(projectFolder, '*.mat');
sync = cellfun(@(X) {load(X)}, sync);

% Get the skeleton
skeleton = load('skeletons/mouse22');
pause
%%
% In case the demo folder uses the dannce.mat data format.
keyFrameList = round(linspace(3, 20));
framesToLabel = [];
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
    for frame = 1:max(keyFrameList)
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
%         if ismember(frame, framesToLabel)
        if any(abs(keyFrameList-frame)<5)
            temp = chromadapt(temp, gray_val);
            if isempty(new_frame)
                new_frame = temp;
            else
                new_frame = cat(4,new_frame, temp);
            end
            framesToLabel = [framesToLabel; frame];
        end
    end
    videos{nVid} = new_frame;
end
delete(gcp('nocreate'))

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
    params = cellfun(@(X) {load(X)}, calibPaths);
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
labelGui.exportDannce('saveFolder', dannceFolder)

%% Rename YYYYMMDD_HHMMSS_Label3D_dannce to label3d_dannce

%% Duplicate Camera
dannceName = [dannceFolder filesep 'label3d_dannce.mat'];
duplicate_camera(dannceName)

%% Run COM prediction (in terminal DGX)

%% merge COM
dannceName = [dannceFolder filesep 'label3d_dannce.mat'];
merge_com_dannce(dannceName)

%% Show Prediction with label data
pred = load([projectFolder 'dannce/predictions 2.mat']);
pred.predictions = rmfield(pred.predictions, 'sampleID');

temp = zeros(22, 3, length(framesToLabel));
for i = 1:length(framesToLabel)
    A = cell2mat(structfun(@(X) {X(framesToLabel(i),:)}, pred.predictions));
    temp(:,:,i) = A;
end

close all
labelGui = Label3D(params, videos, skeleton, 'sync', sync, 'framesToLabel', framesToLabel);
labelGui.loadFrom3D(permute(temp, [3,2,1]))

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

%% Visualize Arena
danncePred = [projectFolder filesep 'prediction' filesep 'save_data_AVG0.mat'];
comPred = [projectFolder filesep 'prediction' filesep 'com3d_used.mat'];
arenaCenter = mean(cell2mat(worldLocation')); 
[viewer, fig, floor, cyl] = viewArena(comPred, danncePred, [arenaCenter(1), arenaCenter(2), 0]);


%% Preprocess data
predictionFile = [projectFolder filesep 'prediction' filesep 'predictions.mat'];
recordingFile = [projectFolder filesep 'recording' filesep 'WT22_220308.mat'];
gpioFile = [projectFolder filesep 'recording' filesep 'WT22_220308_GPIO.csv'];
syncFile = [projectFolder filesep 'recording' filesep 'sync.mat'];
preprocFile = [projectFolder filesep 'prediction' filesep 'preprocessing.mat'];

if ~exist(preprocFile, 'file')
    [poseStruct, recordStruct] = preprocess_prediction(predictionFile, recordingFile, gpioFile, syncFile, skeleton);
    poseStruct.projectFolder = projectFolder;
    save(preprocFile,'poseStruct', 'recordStruct');
else
    load(preprocFile)
end

%%
filename_in = [projectFolder filesep 'prediction' filesep 'predictions.mat'];
filename_out = [projectFolder filesep 'prediction' filesep 'processed_predictions.mat'];
species_name = 'kylemouse2';
input_params.conversion_factor = 5;
input_params.SpineF_marker = 'SpineF';
input_params.SpineM_marker = 'SpineM';
input_params.repfactor = 5;
preprocess_dannce(filename_in,filename_out,species_name,input_params)

pause
%% Compute complex pose feature
preprocDannce = [projectFolder filesep 'prediction' filesep 'processed_predictions.mat'];
mocapstruct = load(preprocDannce);
if isstruct(mocapstruct) && numel(fieldnames(mocapstruct)) == 1
    fname = fieldnames(mocapstruct);
    mocapstruct = mocapstruct.(fname{1});
end

mocapstruct.fps = 300;
coefficient_file = '';
ratnames = 'kyle_mouse';
linkname = 'kyle_mouse';

% CAPTURE Feature filename and whether or not to overwrite
featureFile = [projectFolder filesep 'prediction' filesep 'myMLfeatures.mat'];
coeffFile = [projectFolder filesep 'prediction' filesep 'myWLcoeff.mat'];
overwrite_featureFile = 1;
overwrite_coefficient = 1;

mocapstruct.modular_cluster_properties.clipped_index{8} = 1:size(mocapstruct.aligned_mean_position,1 );

if ~exist(featureFile,'file')
    MLmatobj = create_behavioral_features(mocapstruct,coeffFile,overwrite_coefficient,linkname);
    save(featureFile, 'MLmatobj');
else
    load(featureFile);
end

% MLmatobj_extra = create_extra_behavioral_features(mocapstruct,'myrat','',overwrite_coefficient,[projectFolder filesep 'prediction' filesep 'extra']);
% jt_features_extra = load_extra_tsne_features(mocapstruct,MLmatobj_extra,analysisparams);


tsneFile = [projectFolder filesep 'prediction' filesep 'tsneCluster.mat'];
if ~exist(tsneFile,'file') 
    % perform a tsne embedding subselecting every 50 frames
    analysisparams.tsnegranularity = 30;
    
    %subselect a particular set of features
    analysisstruct = compute_tsne_features(MLmatobj,mocapstruct,analysisparams);
    
    %run tsne
    zvals = tsne(analysisstruct.jt_features);
    analysisstruct.zValues = zvals;
    
    
    % Visualize
    analysisstruct.params.density_res = 1001; %resolution of the map
    analysisstruct.params.density_width = 9; %density kernel in tsne space
    analysisstruct.params.expansion_factor = 1.05; %add a little room to the map after kernel smoothing
    analysisstruct.params.density_threshold = 1*10^(-5); %remove regions in plots with low density
    analysisstruct.matchedconds = {[unique(analysisstruct.condition_inds)]}; %if running over multiple conditions
    analysisstruct.conditions_to_run = [unique(analysisstruct.condition_inds)];
    analysisstruct.tsnegranularity = analysisparams.tsnegranularity;
    tsne_params.reorder=1;
    analysisstruct = compute_analysis_clusters_demo(analysisstruct,tsne_params);
    
    % plot a tsne map -- see plotting script for parameter definitions
    h1=figure(609)
    clf;
    tsne_params.nameplot=1;
    tsne_params.density_plot =0;
    tsne_params.watershed = 1;
    tsne_params.sorted = 1;
    tsne_params.markersize = 5;
    tsne_params.coarseboundary =0;
    tsne_params.do_coarse = 0;
    plot_clustercolored_tsne(analysisstruct,1,tsne_params.watershed,h1,tsne_params)
    set(gcf,'Position',([100 100 1100 1100]))
    
    save(tsneFile,'analysisstruct');
else
    load(tsneFile)
end


%% 
if(0)
    cc = pdist2(poseStruct.filtVelocity(:,poseStruct.framesMove), recordStruct.traceRaw(:,poseStruct.framesMove),'correlation');
    z = linkage(cc', 'weighted');
    c = cluster(z,'maxclust',5);
    [ic,ii] = sort(c);
    imagesc(cc(:,ii)); colormap(flipud(cbrewer('div', 'RdYlBu', 100)))
    yticklabels(fieldnames(poseStruct.predictions))
    yticks(1:21)
    caxis([0.6 1.5])
    
    
    cc = pdist2(poseStruct.filtVelocity(:,poseStruct.framesMove), recordStruct.traceRaw(:,poseStruct.framesMove),'correlation');
    z = linkage(cc', 'weighted');
    c = cluster(z,'maxclust',5);
    [ic,ii] = sort(c);
    markerNames = fieldnames(poseStruct.predictions);
    imagesc(cc([8,9,10,11,16,17,18,12,13,14,15,19,20,21],ii)); colormap(flipud(cbrewer('div', 'RdYlBu', 100)))
    yticklabels(markerNames([8,9,10,11,16,17,18,12,13,14,15,19,20,21]))
    yticks(1:21)
    caxis([0.6 1.5])
end

%% Filter cells
removeId = [10 16 17 18 25 26 27 39 40 41 43 44 46 48 49 50 51 52 53 54];
validId = setdiff(1:size(recordStruct.trace,1), removeId);

recordStruct.valid_trace = recordStruct.trace(validId,:);
recordStruct.valid_traceRaw = recordStruct.traceRaw(validId,:);

%% Per pose correlation
cellClust = {};


analysisstruct.clustered_pose = analysisstruct.annot_reordered{end};
% Scratching stomach
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==1) = 1;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==2) = 1;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==3) = 1;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==4) = 1;
% Still head down
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==5) = 2;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==6) = 2;
% Lean forward
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==7) = 3;
% Head Up
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==8) = 4;
% Forward moving, right turn
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==9) = 5;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==10) = 5;
% Forward moving, left turn
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==12) = 6;
% Standing
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==11) = 7;
% Grooming
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==13) = 8;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==14) = 8;
% Lying down
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==15) = 9;
analysisstruct.clustered_pose(analysisstruct.annot_reordered{end}==16) = 9;

poseStruct.poseCluster = analysisstruct.clustered_pose;
save([projectFolder filesep 'prediction' filesep 'final.mat'], 'poseStruct', 'recordStruct')


poseList = unique(analysisstruct.clustered_pose);

cc = pdist2(poseStruct.filtVelocity(:,:), recordStruct.valid_traceRaw(:,:),'correlation');
z = linkage(cc', 'weighted');
c = cluster(z,'maxclust',5);
[ic,ii] = sort(c);
%%
for poseIdx = poseList
%     figure(1);
    idx = find(analysisstruct.clustered_pose==poseIdx);
    frameList = ceil(analysisstruct.frames_with_good_tracking{1}(idx)/5);
    cc = pdist2(poseStruct.filtVelocity(:,frameList), recordStruct.valid_traceRaw(:,frameList),'correlation');


%     poseInit = [1 find((idx(2:end) - idx(1:end-1)))];
%     poseLength = [poseInit(2:end) length(idx)] - poseInit;
%     for j = 1:10:length(poseLength)
% %         if poseLength(j) < 10
% %             continue
% %         end
%         poseFrames = idx(poseInit(j):poseInit(j)+poseLength(j));
%         poseFrames = poseFrames(1:min(30, length(poseFrames)));
%         animate_markers_nonaligned_fullmovie_demo(analysisstruct.mocapstruct_reduced_agg{1},poseFrames,[],[],'');
%         figure(370);
%         clf;
%     end
    
    cellClust{poseIdx} = cell(21,3);
    for i = 1:21
        cellClust{poseIdx}{i,1} = find(cc(i,:) < 0.6);
        cellClust{poseIdx}{i,2} = find((cc(i,:) > 0.6) & (cc(i,:) < 1.4));
        cellClust{poseIdx}{i,3} = find(cc(i,:) > 1.4);
    end
    
    
%     pause
end

%% 
recordStruct.traceNorm = recordStruct.traceRaw;
for i = 1:size(recordStruct.traceRaw,1)
    recordStruct.traceNorm(i,:) = zscore(recordStruct.traceNorm(i,:));
end

recordStruct.valid_traceNorm = recordStruct.valid_traceRaw;
for i = 1:size(recordStruct.valid_traceRaw,1)
    recordStruct.valid_traceNorm(i,:) = zscore(recordStruct.valid_traceNorm(i,:));
end


% Currently at 12 Hz
numPose = max(poseList);
transitionTrace = cell(numPose,numPose);
cmap = cbrewer('qual','Paired', numPose);
poseInit = [0 find(diff(analysisstruct.clustered_pose))] + 1;
poseLength = [poseInit(2:end) length(analysisstruct.clustered_pose)] - poseInit;

for j = 1:length(poseInit)
    if((poseInit(j) < 6) || (poseInit(j) > numel(analysisstruct.clustered_pose)-5))
        continue
    end
    poseFrames = poseInit(j)-6:poseInit(j)+5;
    frameList = (analysisstruct.frames_with_good_tracking{1}(poseFrames)-1)/5+1;
    frameList = min(frameList):max(frameList);
    
    currPose = analysisstruct.clustered_pose(poseInit(j)-1);
    nextPose = analysisstruct.clustered_pose(poseInit(j));
    transitionTrace{currPose, nextPose} = cat(1,transitionTrace{currPose, nextPose}, smoothdata(recordStruct.valid_traceNorm(:,frameList),2,'movmean', 5)');
end

%% Pose-wise Activity
poseActivity = zeros(numPose, size(recordStruct.valid_traceRaw,1));
% poseTrace = 
poseFrame = cell(numPose,1);
for poseIdx = 1:numPose
    idx = find(analysisstruct.clustered_pose==poseIdx);
    frameList = (analysisstruct.frames_with_good_tracking{1}(idx)-1)/5 + 1;
    poseInit = [0 find(abs(diff(idx))>5)] + 1;
    poseLength = [poseInit(2:end) length(idx)] - poseInit;

    poseFrame{poseIdx} = [];
    for j = 1:length(poseInit)
        segFrame = frameList(poseInit(j):poseInit(j)+poseLength(j)-1);
        segFrame = min(segFrame):max(segFrame);
        poseFrame{poseIdx} = [poseFrame{poseIdx}; segFrame'];
    end
    
    poseActivity(poseIdx,:) = mean(recordStruct.valid_traceNorm(:,poseFrame{poseIdx}),2);
%     plot(recordStruct.valid_traceNorm(:,poseFrame{poseIdx})')
%     pause
end


%% Pose Transition
for i = 1:numPose
    for j = 1:numPose
        if size(transitionTrace{i,j},1) == 0
            fprintf('Empty\n')
            continue
        end
        if size(transitionTrace{i,j},1) > 12
            fprintf('Current transition from Pose %d to %d\n', i,j)
            temp = permute(reshape(transitionTrace{i,j}, [67, size(transitionTrace{i,j},1)/67, numel(validId)]), [1 3 2]);
            temp = mean(temp,3);
        else 
            fprintf('Single\n')
            temp = transitionTrace{i,j};
        end
        figure(1)
        subplot(2,1,1)
        imagesc([i*ones(1,6) j*ones(1,6)])
        colormap(cmap)
        axis off
        colorbar
        caxis([1 numPose]+0.5)
        subplot(2,1,2)
        ax = imagesc(temp');
        caxis([-5 5])
        colorbar
        colormap(ax.Parent, cbrewer('seq', 'Greens',100))
        sgtitle(sprintf('Current transition from Pose %d to %d', i,j))
%         pause
    end
end

%% Check Each Cluster
activePose = [2 3 8];
passivePose = setdiff(poseList, activePose);

for i = 1:numPose    
    for j = 1:numPose  
        if isempty(transitionTrace{i,j})
            continue
        end
        tempTrace = permute(reshape(transitionTrace{i,j}, [67, size(transitionTrace{i,j},1)/67, numel(validId)]), [1 3 2]);
        ax = imagesc(mean(tempTrace(:,ii,:),3)');
        yticks([0 cumsum(histcounts(ic))]);
        sgtitle(sprintf('Current transition from Pose %d to %d', i,j))
        colormap(ax.Parent, flipud(cbrewer('div', 'RdYlBu',100)))
        caxis([-4 4])
%         pause
    end
end


%% Figures
saveFolder = ['/Users/hyungju/Desktop/hyungju/Result/optodbs-2022/pdf'];
saveFig = true;

% Figure 1
close all
subplot(2,1,1)
imagesc(poseStruct.poseCluster)
colormap(cmap); caxis([0.5 +numPose+0.5])
colorbar
axis off
subplot(2,1,2)
z = linkage(pdist(recordStruct.valid_traceNorm,'seuclidean'), 'ward');
c = cluster(z,'maxclust',5);
[ic,ii] = sort(c);
ax = imagesc(recordStruct.valid_traceNorm(ii,:));
colorbar
colormap(ax.Parent, flipud(cbrewer('div', 'RdBu',100)))
caxis([-4 4])
yticks([0 cumsum(histcounts(ic))]);

if(saveFig)
    figureFolder = [saveFolder filesep 'normalized_activity_pose'];
    mkdir(figureFolder)
    pdfName = [figureFolder filesep '0308_wt_22_normalized_activity_pose'];
    save2pdf(pdfName)
    close all
end

zz = linkage(poseActivity, 'complete','cosine');
cz = cluster(zz,'maxclust',4);
[icz,iiz] = sort(cz);
imagesc(poseActivity(iiz,ii)')
colormap(flipud(cbrewer('div', 'RdBu',100)))
yticks([0 cumsum(histcounts(ic))]);
xticks([0 cumsum(histcounts(icz))]);
caxis([-1 1])

if(saveFig)
    figureFolder = [saveFolder filesep 'normalized_activity_pose'];
    mkdir(figureFolder)
    pdfName = [figureFolder filesep sprintf('0308_wt_22_average_activity_pose', i, j)];
    save2pdf(pdfName)
    close all
end

%%
% Figure 2
for poseIdx = 1:numPose
    idx = find(analysisstruct.clustered_pose==poseIdx);
    frameList = (analysisstruct.frames_with_good_tracking{1}(idx)-1)/5 + 1;
    poseInit = [0 find(abs(diff(idx))>5)] + 1;
    poseLength = [poseInit(2:end) length(idx)] - poseInit;
    poseFrame{poseIdx} = [];
    for j = 1:length(poseInit)
        segFrame = frameList(poseInit(j):poseInit(j)+poseLength(j)-1);
        segFrame = min(segFrame):max(segFrame);
        poseFrame{poseIdx} = [poseFrame{poseIdx}; segFrame'];
    end
    cc = pdist2(poseStruct.filtRelVelocity(:,poseFrame{poseIdx}), recordStruct.valid_traceRaw(:,poseFrame{poseIdx}),'correlation');
  
    ccc = [mean(cc(8:10,ii), 1); mean(cc(12:14, ii), 1)];  
    subplot(1,3,[1 2])
    bar(ccc')
    xticks([1 cumsum(histcounts(ic))]);
    
    subplot(1,3,3)
    ccc = zeros(2,5);
    for j = 1:5
        ccc(:,j) = [mean(mean(cc(8:10,c==j), 2)); mean(mean(cc(12:14, c==j), 2))];
    end
    bar(zscore(ccc)')
    sgtitle(sprintf('Pose %d, Upper', poseIdx))
    if(saveFig)
        figureFolder = [saveFolder filesep 'corr_cell_group_pose'];
        mkdir(figureFolder)
        pdfName = [figureFolder filesep sprintf('0308_wt_22_upper_pose_%d', poseIdx)];
        save2pdf(pdfName)
        close all
    end
    ccc = [mean(cc(16:18,ii), 1); mean(cc(19:21, ii), 1)];
    subplot(1,3,[1 2])
    bar(ccc')
    xticks([1 cumsum(histcounts(ic))]);
    
    subplot(1,3,3)
    ccc = zeros(2,5);
    for j = 1:5
        ccc(:,j) = [mean(mean(cc(16:18,c==j), 2)); mean(mean(cc(19:21, c==j), 2))];
    end
    bar(zscore(ccc)')
    sgtitle(sprintf('Pose %d, Lower', poseIdx))
    
    if(saveFig)
        figureFolder = [saveFolder filesep 'corr_cell_group_pose'];
        mkdir(figureFolder)
        pdfName = [figureFolder filesep sprintf('0308_wt_22_lower_pose_%d', poseIdx)];
        save2pdf(pdfName)
        close all
    end
    ccc = [mean(cc([8:10 16:18],ii), 1); mean(cc([12:14 19:21], ii), 1)];
    subplot(1,3,[1 2])
    bar(ccc')
    xticks([1 cumsum(histcounts(ic))]);
    
    subplot(1,3,3)
    ccc = zeros(2,5);
    for j = 1:5
        ccc(:,j) = [mean(mean(cc([8:10 16:18],c==j), 2)); mean(mean(cc([12:14 19:21], c==j), 2))];
    end
    bar(zscore(ccc)')
    sgtitle(sprintf('Pose %d, Both', poseIdx))
    
    if(saveFig)
        figureFolder = [saveFolder filesep 'corr_cell_group_pose'];
        mkdir(figureFolder)
        pdfName = [figureFolder filesep sprintf('0308_wt_22_both_pose_%d', poseIdx)];
        save2pdf(pdfName)
        close all
    end
end

%% Figure 3
for i = 1:numPose    
    for j = 1:numPose  
        if isempty(transitionTrace{i,j})
            continue
        end
        tempTrace = permute(reshape(transitionTrace{i,j}, [67, size(transitionTrace{i,j},1)/67, numel(validId)]), [1 3 2]);
        ax = imagesc(mean(tempTrace(:,ii,:),3)');
        yticks([0 cumsum(histcounts(ic))]);
        sgtitle(sprintf('Current transition from Pose %d to %d', i,j))
        colormap(ax.Parent, flipud(cbrewer('div', 'RdBu',100)))
        caxis([-4 4])
        
        if(saveFig)
            figureFolder = [saveFolder filesep 'transition_cells'];
            mkdir(figureFolder)
            pdfName = [figureFolder filesep sprintf('0308_wt_22_transition_from_%d_to_%d', i, j)];
            save2pdf(pdfName)
            close all
        end
    end
end

%% Per cell per body part moving window correlation
for bodyIdx = 1:21
    timeCorr = zeros(numel(validId), numel(poseStruct.sampleID)-120);
    for frameIdx = 61:numel(poseStruct.sampleID)-60
        timeCorr(:,frameIdx-60) = pdist2(poseStruct.filtRelVelocity(bodyIdx,frameIdx-60:frameIdx+60), recordStruct.valid_traceRaw(:,frameIdx-60:frameIdx+60),'correlation');
    end
    for i = 1:35
        subplot(35,1,i)
        plot(timeCorr(ii(i),:)')
        axis off
        if(saveFig)
            figureFolder = [saveFolder filesep 'body_time_correlation'];
            mkdir(figureFolder)
            pdfName = [figureFolder filesep sprintf('0308_wt_22_transition_from_%d_to_%d', i, j)];
            save2pdf(pdfName)
            close all
        end
    end
    pause
end
    

%%
imshow(recordStruct.neuron.Cn)
hold on
for i = 1:5
    for j = 1:length(validId(ii(ic == i)))
        temp = validId(ii(ic == i));
        plot(recordStruct.neuron.Coor{temp(j)}(1,:), recordStruct.neuron.Coor{temp(j)}(2,:), 'color', setColor(i,:), 'LineWidth', 2)
    end
end


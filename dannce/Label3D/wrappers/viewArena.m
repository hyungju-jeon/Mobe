function [viewer, fig, floor, cyl] = viewArena(comPath, danncePath, varargin)
% viewArena - View simple arena schematic for easy visualization of COMs.
%
% Inputs: comPath - path to COM file.
%         danncePath - path to dannce predictions file. 
%
% Outputs: viewer - cell array of animators in viewer
%          fig - figure handle
%          floor - handle to floor patch
%          cyl - handle to cylinder surface.
%
% Example:
% comPath = '/home/diego/cluster/Jesse/P21_pups/RecordingP21Pup_one/COM/predict_results_test_01092020MAX/COM3D_undistorted_medfilt.mat';
% danncePath = '/home/diego/cluster/Jesse/P21_pups/RecordingP21Pup_one/DANNCE/predict_results_test_01092020MAX/save_data_MAX.mat';
% viewArena(comPath, danncePath);
%% Args
center = [0, 0, 0];
start_frame = 1;
if ~isempty(varargin)
    center = varargin{1};
    if numel(varargin) == 2
        start_frame = varargin{2};
    end
end
%% Loading
COMs = load(comPath);
pred = load(danncePath);
nFrames = size(pred.pred,1);
pred.pred(:,3,:) = -pred.pred(:,3,:);
com = COMs.com(1:nFrames-1,:);
com = COMs.com(start_frame:start_frame+nFrames-1,:);
skeleton = load('skeletons/mouse22');
skeleton.joint_names(8) = [];
skeleton.joints_idx(10,:) = [];
skeleton.color(10,:) = [];
% skeleton = load('mouse_22.mat');
%% Make the box
% box = zeros(size(com,1), size(com,2), numel(skeleton.joint_names));
% vMin = -80;
% vMax = 80;
% box(:,:,1) = com + [vMin vMax vMax];
% box(:,:,2) = com + [vMax vMax vMax];
% box(:,:,3) = com + [vMax vMin vMax];
% box(:,:,4) = com + [vMin vMin vMax];
% box(:,:,5) = com + [vMin vMax vMin];
% box(:,:,6) = com + [vMax vMax vMin];
% box(:,:,7) = com + [vMax vMin vMin];
% box(:,:,8) = com + [vMin vMin vMin];
% box(:,:,9) = com;
% box = box - center;

%% Get the 3d pos
% data_3d = smoothdata(smoothdata(pred.pred + com,'movmedian',3),'gaussian',5);
data_3d = smoothdata(pred.pred,'gaussian',2);
data_3d = data_3d - center;

%% Make the figure;
fig = figure('pos',[675 195 862 774]);
set(fig,'color','k');
arenaSize = 150;
arenaHeight = 0;
viewer{1} = Keypoint3DAnimator(data_3d, skeleton);
ax = viewer{1}.Axes;
% viewer{2} = Keypoint3DAnimator(box, boxskeleton, 'Axes', ax);
Animator.linkAll(viewer)
view(ax, 3)
lim = [-200 200];
set(ax,'XGrid','on','YGrid','on','ZGrid','on','XLim',lim, 'YLim',lim, 'ZLim',[-10 250])
daspect(ax,[1 1 1])
floor = patch(ax, lim([0 1 1 0]+1),lim([0 0 1 1]+1),[0 0 0 0],[.5 .5 .5],'FaceAlpha',.5);
[x,y,z] = cylinder(arenaSize);
cyl = surf(ax, x,y,(z)*arenaHeight,'FaceAlpha',.1,'FaceColor',[1 1 1],'EdgeColor','none');
axis off
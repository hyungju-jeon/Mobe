
%% Use predetermined crop ROI parameters to crop a single image of the
% calibration lframe We will use these
% cropped images to determine view extrinsic parameters.
% clear
% Load in the crop params
basedir = '/Users/hyungju/Desktop/hyungju/Result/functional-stn/220831-takeOver/calibration'
numCams = 4;
numPoints = 9;
ext = '.tiff';

% Input the physical (x,y,z) dimensions for each of the post tops on the 
% L-frame device.
% LFrame_coordinates = [ -5 -5 2.5; 
%                         5 -5 4.5; 
%                        -5 5 6.5; 
%                         0 0 8.5; 
%                         5 5 10.5];
% stepSize = 2.4 % Old version
stepSize = 1.5 % new version                   
LFrame_coordinates = [ 0    0   0; 
                       1    0   0; 
                       2    0   0;
                       3    0   0;
                       3    1   0;
                       3    2   0;
                       3    3   0; 
                       4    3   0;
                       5    3   0]*stepSize;
LFrame_coordinates = 10*(LFrame_coordinates); % cm to mm

%convert first frame of each video to tiff
% fileloc = [basedir filesep 'extrinsic'];
% cd(fileloc)
for kk = 1:numCams
    filename = [basedir filesep 'videos' filesep 'Camera' num2str(kk) filesep 'view_' num2str(kk)];
%     video_temp = VideoReader([basedir 'extrinsic' filesep filename '.mp4'],'CurrentTime', 0);
    video_temp = VideoReader([basedir  filesep 'videos' filesep 'Camera' num2str(kk) filesep  '0.mp4']);
    frame_temp = readFrame(video_temp);
    %imshow(frame_temp)
    imwrite(frame_temp,[filename,'.tiff'])
end

lframe = cell(numCams,1);
for i = 1:numCams
    lframe{i} = imread([basedir  filesep 'videos' filesep 'Camera' num2str(i) filesep 'view_' num2str(i) ext]);
end
% load([basedir 'intrinsic' filesep 'cam_intrinsics.mat']);
% focalLength = {[681.064, 679.49], [681.956, 680.916], [683.922, 683.364], [681.56, 680.097]};
% principalPoint = {[485.075, 285.476], [487.072, 276.444], [479.709, 275.543], [491.356, 274.026]};
focalLength = {[683.922, 683.364], ...  % camera_034422070939 -left
               [681.56,  680.097], ...  % camera_104422070051 -down
               [681.956, 680.916], ...  % camera_104422070056 -up 
               [681.064, 679.49]};      % camera_104422070104 -right
principalPoint = {[479.709, 275.543], ...   % camera_034422070939 -left
                  [491.356, 274.026], ...   % camera_104422070051 -down
                  [487.072, 276.444], ...   % camera_104422070056 -up  
                  [485.075, 285.476]};      % camera_104422070104 -right 
% focalLength = focalLength([4,3,1,2]);
% principalPoint = principalPoint([4,3,1,2]);
imageSize = size(frame_temp,[1,2]);

intrinsic = cell(numCams,1);
params_individual = cell(numCams,1);
for i = 1:numCams
    intrinsics{i} = cameraIntrinsics(focalLength{i}, principalPoint{i}, imageSize);
    params_individual{i} = cameraParameters('IntrinsicMatrix', intrinsics{i}.IntrinsicMatrix, 'ImageSize', imageSize);
end

%% Click points for each post top in order for each view
figure;
for kk = 1:numel(lframe)
    camparams = params_individual{kk};
    LFrame_image{kk} = undistortImage(lframe{kk},intrinsics{kk});
    imagesc(LFrame_image{kk});colormap(gray)
    [xi,yi] = getpts;
    point_coordinates{kk} = [xi yi];
end

% Remove any extra points at the end that were accidentally marked
for kk = 1:numel(lframe)
    point_coordinates{kk} = point_coordinates{kk}(1:numPoints,:);
end

% Plot point coordinates
% figure;
% for kk = 1:numel(lframe)
%     hold off; imagesc(LFrame_image{kk});colormap(gray)
%     hold on; plot(point_coordinates{kk}(:,1),point_coordinates{kk}(:,2),'or')
%     waitforbuttonpress;
% end

% If we mis-labeled a point, remove it here
% rmf = 5;
% for kk = 1:numel(lframe)
%     point_coordinates{kk}(4,:) = [];
% end

% Also remove from LFrame_coordinates
%LFrame_coordinates(5:end,:) = [];

%% Use selected points to calculate camera extrinsics
for kk = 1:numel(lframe)
    %kk
    % Do grid search over parms
    curr_err = 1e10;
    c_save = 0;
    mr_save = 0;
    mn_save = 10;
    for c = 99:-3:90
        %c
        for mr = [0.5 1 1.5 2 2.5 3 3.5 4 4.5 5:5:50]
            for mn = 1e10 
                try
                    [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                        double(LFrame_coordinates),params_individual{kk},'Confidence',c,'MaxReprojectionError',mr,'MaxNumTrials',mn);
                    for i = 1:200
                        [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                            double(LFrame_coordinates),params_individual{kk},'Confidence',c,'MaxReprojectionError',mr,'MaxNumTrials',mn);
                        [rotationM,translationV] = cameraPoseToExtrinsics(worldO,worldL);
                        imagePoints = worldToImage(params_individual{kk},rotationM,translationV,double(LFrame_coordinates));
                        err = mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)));
                        if err < curr_err
                            ["Found better repro error: " num2str(err)]
                            curr_err = err;
                            c_save = c;
                            mr_save = mr;
                            mn_save = mn;
                        end
                        
                    end
                end
            end
        end
    end
    
    curr_err = 1e10;
    for i=1:200
        try
            [worldO,worldL] = estimateWorldCameraPose(double(point_coordinates{kk}),...
                double(LFrame_coordinates),params_individual{kk},'Confidence',c_save,'MaxReprojectionError',mr_save,'MaxNumTrials',mn_save);
            [rotationM,translationV] = cameraPoseToExtrinsics(worldO,worldL);
            imagePoints = worldToImage(params_individual{kk},rotationM,translationV,double(LFrame_coordinates));
            err = mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)));
            if err < curr_err
                rotationMatrix{kk} = rotationM;
                translationVector{kk} = translationV;
                worldOrientation{kk} = worldO;
                worldLocation{kk} = worldL;
                curr_err = err;
            end
        end
    end
    
    figure(222)
    plotCamera('Location',worldLocation{kk},'Orientation',worldOrientation{kk},'Size',50,'Label',num2str(kk));
    hold on
end
% print('-dpng',[basedir 'cameraArrangement.png']);

% Save full camera parameters: intrinsics, extrinsics
save_folder = [basedir filesep 'extrinsic'];
mkdir(save_folder)
save([save_folder filesep 'camera_params'],'params_individual','worldOrientation','worldLocation','rotationMatrix','translationVector');

%% Find closed form solution for rotation and translation.
% for kk = 1:numel(lframe)
% [rotationMatrix{kk}, translationVector{kk}] = extrinsics(double(point_coordinates{kk}),double(LFrame_coordinates(:,1:2)),params_individual{kk});
% end
%% Examine reprojections
%y_reflect = [-1 0 0; 0 -1 0; 0 0 1];
for kk = 1:numel(lframe)
    figure(233+kk)
    imagesc( LFrame_image{kk});colormap(gray);
    hold on
    
    tmp_v = translationVector{kk};
    % tmp_v(3) = -1*tmp_v(3);
    %tmp_v = -1*tmp_v;
    %     tmp_v
    tmp_r = rotationMatrix{kk};
    % tmp_r(:,1) = -1*tmp_r(:,1);
    imagePoints = worldToImage(params_individual{kk},tmp_r,tmp_v,double(LFrame_coordinates));
    
    colorarray = jet(size(imagePoints,1));%{'r','g','b','y','m','w','k'};
    for llll = 1:size(imagePoints,1)
        plot(imagePoints(llll,1),imagePoints(llll,2),'o','MarkerSize',4,...
            'MarkerEdgeColor',colorarray(llll,:),'MarkerFaceColor',colorarray(llll,:))
    end
    for llll = 1:size(imagePoints,1)
        plot(point_coordinates{kk}(llll,1),point_coordinates{kk}(llll,2),'x','MarkerSize',4,...
            'MarkerEdgeColor',colorarray(llll,:),'MarkerFaceColor',colorarray(llll,:))
    end
    hold off
%     print('-dpng',[basedir 'camerareproject',num2str(kk),'.png']);
    mean(sqrt(sum((imagePoints-point_coordinates{kk}).^2,2)))
end

%%camera to world image
for kk = 1:numCams
     figure(222)
     plotCamera('Location',worldLocation{kk},'Orientation',worldOrientation{kk},'Size',5,'Label',num2str(kk));
     hold on;
end

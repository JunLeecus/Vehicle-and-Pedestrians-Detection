
focalLength    = [1260 1100];
principalPoint = [360 245];
imageSize = [480,640]; 
camintrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
height = 1.45;   
pitch  = 1.25;   
sensor = monoCamera(camintrinsics, height,'Pitch', pitch);

videoNmae   = '交通道路视频.mp4';
videoReader = VideoReader(videoNmae);

writerObj=VideoWriter('test.avi');  %// 定义一个视频文件用来存动画

timeStamp = 1.5;
videoReader.CurrentTime = timeStamp;
frame = readFrame(videoReader);
 

distAheadOfSensor = 30; 
spaceToOneSide = 6;
bottomOffset = 3;

outView = [bottomOffset,distAheadOfSensor,-spaceToOneSide,spaceToOneSide];
imageSize = [NaN,250]; 
birdsEyeConfig = birdsEyeView(sensor,outView,imageSize);

birdsEyeImage = transformImage(birdsEyeConfig,frame);


birdsEyeImage = rgb2gray(birdsEyeImage);

vehicleROI=outView-[-1,2,-3,3];
approxLaneMarkerWidthVehicle=0.25;
laneSensitivity = 0.25;
birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage,birdsEyeConfig,approxLaneMarkerWidthVehicle,... 
    'ROI',vehicleROI,'Sensitivity',laneSensitivity);


[imageX, imageY] = find(birdsEyeViewBW);
xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
maxLanes      = 2; 
boundaryWidth = 3*approxLaneMarkerWidthVehicle; 

[boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
    'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; 


isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
boundaries    = boundaries(isOfMinLength);

maxPossibleXLength = diff(vehicleROI(1:2));
minXLength         = maxPossibleXLength * 0.60; 

isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
boundaries    = boundaries(isOfMinLength);

birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
[laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));

vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);

maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));


maxLaneLength = diff(vehicleROI(1:2));

maxStrength   = maxPointsInOneLane/maxLaneLength;

isStrong      = [boundaries.Strength] > 0.4*maxStrength;
boundaries    = boundaries(isStrong);
boundaries = classifyLaneTypes(boundaries, boundaryPoints);

xOffset    = 0;   
distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);

leftEgoBoundaryIndex  = [];
rightEgoBoundaryIndex = [];
minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
if ~isempty(minLDistance)
    leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
end
if ~isempty(minRDistance)
    rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
end
leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);

videoReader.CurrentTime = 0;
isPlayerOpen = true;
snapshot     = [];
open(writerObj);
while hasFrame(videoReader) && isPlayerOpen

    frame = readFrame(videoReader);

    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);

    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig, ...
        approxLaneMarkerWidthVehicle, 'ROI', vehicleROI, ...
        'Sensitivity', laneSensitivity);

    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);

    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);

    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);

    boundaries = classifyLaneTypes(boundaries, boundaryPoints);

    xOffset    = 0;  
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
  
    leftEgoBoundaryIndex  = [];
    rightEgoBoundaryIndex = [];
    minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
    minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
    if ~isempty(minLDistance)
        leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
    end
    if ~isempty(minRDistance)
        rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
    end
    leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
    rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);


% Call vehicleDetectorACF to detect vechicle
% Call peopleDetectorACF to detect pedastrain
    vehicleWidth = [1.5, 2.5];
    monoDetector = configureDetectorMonoCamera(vehicleDetectorACF(), sensor, vehicleWidth);
    [bboxes1, scores1] = detect(monoDetector, frame);
    [bboxes2, scores2] = detect(peopleDetectorACF, frame);
% merge the bboxes and scores
    bboxes = [bboxes1;bboxes2];
    scores = [scores1;scores2];
% locate all bboxes
    locations = computeVehicleLocations(bboxes, sensor);

    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;

    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;

    intOut.birdsEyeImage   = birdsEyeImage;
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;

    closePlayers = ~hasFrame(videoReader);
    
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers, writerObj);

    timeStamp = 2; 
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
    
end
close(writerObj);
if ~isempty(snapshot)
    figure
    imshow(snapshot)
end
function isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut,...
    intOut, closePlayers, writerObj)

    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;

    xVehiclePoints   = sensorOut.xVehiclePoints;
    bboxes           = sensorOut.vehicleBoxes;

    birdsEyeViewImage = intOut.birdsEyeImage;
    birdsEyeConfig    = intOut.birdsEyeConfig;
    vehicleROI        = intOut.vehicleROI;
    birdsEyeViewBW    = intOut.birdsEyeBW;

    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');

    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');

    frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);
    
    writeVideo(writerObj,frameWithOverlays);
    
    imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];

    birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI);
    birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'blue');

    frames = {frameWithOverlays, birdsEyeViewImage, birdsEyeWithOverlays};

    persistent players;
    if isempty(players)
        frameNames = {'Lane marker and vehicle detections', 'Raw segmentation', 'Lane marker detections'};
        players = helperVideoPlayerSet(frames, frameNames);
    end
    update(players, frames);

    isPlayerOpen = isOpen(players, 1);

    if (~isPlayerOpen || closePlayers) 
        clear players;
    end
    
end
function locations = computeVehicleLocations(bboxes, sensor)

locations = zeros(size(bboxes,1),2);
for i = 1:size(bboxes, 1)
    bbox  = bboxes(i, :);

    yBottom = bbox(2) + bbox(4) - 1;
    xCenter = bbox(1) + (bbox(3)-1)/2;

    locations(i,:) = imageToVehicle(sensor, [xCenter, yBottom]);
end
end
function imgOut = insertVehicleDetections(imgIn, locations, bboxes)

imgOut = imgIn;

for i = 1:size(locations, 1)
    location = locations(i, :);
    bbox     = bboxes(i, :);

    label = sprintf('X=%0.2f, Y=%0.2f', location(1), location(2));

    imgOut = insertObjectAnnotation(imgOut, ...
        'rectangle', bbox, label, 'Color','g');
end
end
function imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI)

vehicleROI = double(vehicleROI);

loc2 = abs(vehicleToImage(birdsEyeConfig, [vehicleROI(2) vehicleROI(4)]));
loc1 = abs(vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)]));
loc4 =     vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(4)]);
loc3 =     vehicleToImage(birdsEyeConfig, [vehicleROI(1) vehicleROI(3)]);

[minRoiX, maxRoiX, minRoiY, maxRoiY] = deal(loc4(1), loc3(1), loc2(2), loc1(2));

imageROI = round([minRoiX, maxRoiX, minRoiY, maxRoiY]);

end
function isGood = validateBoundaryFcn(params)

if ~isempty(params)
    a = params(1);
    isGood = abs(a) < 0.003; 
else
    isGood = false;
end
end
function boundaries = classifyLaneTypes(boundaries, boundaryPoints)

for bInd = 1 : numel(boundaries)
    vehiclePoints = boundaryPoints{bInd};

    vehiclePoints = sortrows(vehiclePoints, 1);

    xVehicle = vehiclePoints(:,1);
    xVehicleUnique = unique(xVehicle);


    xdiff  = diff(xVehicleUnique);
   
    xdifft = mean(xdiff) + 3*std(xdiff);
    largeGaps = xdiff(xdiff > xdifft);


    boundaries(bInd).BoundaryType= LaneBoundaryType.Solid;
    if largeGaps>2
     
        boundaries(bInd).BoundaryType = LaneBoundaryType.Dashed;
    end
end

end
function I = takeSnapshot(frame, sensor, sensorOut)

    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;
    xVehiclePoints   = sensorOut.xVehiclePoints;
    bboxes           = sensorOut.vehicleBoxes;

    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
    frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);

    I = frameWithOverlays;

end

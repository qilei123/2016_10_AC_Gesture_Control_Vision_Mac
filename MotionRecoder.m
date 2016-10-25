clear;

FrameRate = 30;
TimeOflength = 10; %s

totalLength = FrameRate *TimeOflength;

writeVedio = VideoWriter('motionVedio.avi');
writeVedio.FrameRate = 30;
open(writeVedio);

cam = webcam();
cam.Resolution = '640x480';

videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object. 
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
frameCount = 0;

while runLoop && frameCount < totalLength
    frameCount = frameCount+1;
    timeStamp(frameCount) = now;
    videoFrame = snapshot(cam);
    step(videoPlayer, videoFrame);
    writeVideo(writeVedio,videoFrame);
    runLoop = isOpen(videoPlayer);
end
save('timeStamp.mat','timeStamp');
clear cam;
close(writeVedio);
release(videoPlayer);
displayEndOfDemoMessage(mfilename);
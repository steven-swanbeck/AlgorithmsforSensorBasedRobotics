clc; clear all; format compact;

% mdl_panda
tpts = 0:4;
sampleRate = 20;
tvec = tpts(1):1/sampleRate:tpts(end);
numSamples = length(tvec);
robot = loadrobot("frankaEmikaPanda",DataFormat="column");

rng default
frankaWaypoints = [robot.homeConfiguration robot.randomConfiguration robot.randomConfiguration];
frankaTimepoints = linspace(tvec(1),tvec(end),3);
[q,qd] = trapveltraj(frankaWaypoints,numSamples);

figure
set(gcf,"Visible","on");
rc = rateControl(sampleRate);
for i = 1:numSamples
    show(robot,q(:,i),FastUpdate=true,PreservePlot=false);
    waitfor(rc);
end

% helperPlotJointSpaceTraj("Joint-Space Trajectory and Waypoints", ...
%     tvec,q,qd,frankaWaypoints,frankaTimepoints);

frankaSpaceWaypoints = [0.5 0.25 0.25; 0.75 0 0.35; 0.5 -0.25 0.25; 0.5 0.25 0.25]';
frankaTimepoints = linspace(tvec(1),tvec(end),4);
[pos,vel] = minjerkpolytraj(frankaSpaceWaypoints,frankaTimepoints,numSamples);

rng(0) % Seed the RNG so the inverse kinematics solution is consistent
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.AllowRandomRestart = false;
q = zeros(9,numSamples);
weights = [0.2 0.2 0.2 1 1 1]; % Prioritize position over orientation
initialGuess = [0, 0, 0, -pi/2, 0, 0, 0, 0.01, 0.01]'; % Choose an inital guess within the robot joint limits
for i = 1:size(pos,2)
    targetPose = trvec2tform(pos(:,i)')*eul2tform([0, 0, pi]);
    q(:,i) = ik("panda_hand",targetPose,weights,initialGuess);
    initialGuess = q(:,i); % Use the last result as the next initial guess
end

figure
set(gcf,"Visible","on")
show(robot);

rc = rateControl(sampleRate);
for i = 1:numSamples
    show(robot, q(:,i),FastUpdate=true,PreservePlot=false);
    waitfor(rc);
end
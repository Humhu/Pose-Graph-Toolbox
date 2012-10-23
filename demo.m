% Demo script
w1 = World2D([1,1]);
w1.InitRobots(3);
w1.PlotPoses
cov = zeros(2);

pause;

[ranges, angles] = w1.GetMeasurements(cov);
w1.PlotMeasurements(ranges, angles);

pause;

w2 = World2D([1,1]);
w2.InitRobots(3);
cov = 1E-1*eye(3);
noise = mvnrnd(zeros(3,3), cov);
noise = reshape(noise, 3, 1, 3);
poses = w1.robot_poses + Pose2D(noise(:,:,1:2), noise(:,:,3));
info = (1E-3*eye(2))^-1;
for i = 1:10
   poses = GaussNewtonIterate(poses, ranges, angles, info);
   w2.robot_poses = poses;
   w2.PlotPoses
   w2.PlotMeasurements(ranges, angles);
   pause;
end


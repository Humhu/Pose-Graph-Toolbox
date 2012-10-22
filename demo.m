pos = InitRobots(N, [1;1]);
cov_mat = cov*eye(2);
meas = GetMeasurements(pos, cov_mat);
est = bsxfun(@plus, pos(:,1), EstimatePositions(pos, meas));

err = pos - est;
avg_err = mean(sqrt(sum(err.*err, 1)));
fprintf(['Average error: ', num2str(avg_err), '\n']);

figure;
plot(pos(1,:), pos(2,:), 'bx');
hold on;
plot(est(1,:), est(2,:), 'ro');
xlim([0, 1]);
ylim([0, 1]);
function [newPoses] = SolverIterate(poses, ranges, angles, info)

errors = CalculateError(poses, ranges, angles);
newPoses = GaussNewtonIterate(poses, errors, info);
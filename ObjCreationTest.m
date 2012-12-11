% Test script to check speed of orientation objects
%% Creation test
num_creations = 10;
ta1 = 2*pi*rand(1,num_creations);
ta2 = 2*pi*rand(1,num_creations);

% Test creating Orientation1D
o1 = Orientation1D(ta1);
o2 = Orientation1D(ta2);

% Test creating Quat2Ds
q1 = Quaternion2D(ta1);
q2 = Quaternion2D(ta2);

%% Addition test
ta3 = wrapToPi(ta1 + ta2);

o3 = o1 + o2;
if any(abs(double(o3) - ta3) > 1E-6)
   fprintf('o3 add error\n'); 
end

q3 = q1 + q2;
if any(abs(double(q3) - ta3) > 1E-6)
   fprintf('q3 add error\n'); 
end

%% Subtraction test
ta4 = wrapToPi(ta1 - ta2);
o4 = o1 - o2;
if any(abs(double(o4) - ta4) > 1E-6)
   fprintf('o3 sub error\n'); 
end

q4 = q1 - q2;
if any(abs(double(q4) - ta4) > 1E-6)
   fprintf('q3 sub error\n'); 
end
%% Conversion to double test
o1d = double(o1);

q1d = double(q1);
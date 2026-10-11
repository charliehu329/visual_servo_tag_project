function tests = test_two_stage_qdot
% Pure-function tests: no ROS node, publisher, or hardware model is run.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'control'));
testCase.TestData.root = root;
end

function testGuardDoesNotRetaperAdmissiblePositionVelocity(testCase)
q = zeros(7,1); q(1) = 0.925;
lo = -ones(7,1); hi = ones(7,1);
vmax = 0.20*ones(7,1); amax = ones(7,1);
previous = zeros(7,1); previous(1) = 0.05;
request = previous;
[command,~,~,~,valid,intervened] = qdot_final_guard( ...
    request,q,previous,true,0.01,0.15,vmax,amax,lo,hi);
verifyTrue(testCase,valid);
verifyFalse(testCase,intervened);
verifyEqual(testCase,command,request,'AbsTol',1e-12);
end

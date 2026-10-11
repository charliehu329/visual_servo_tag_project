%% test_q_dot_analysis_scripts.m
% Regression smoke test for the q_dot post-simulation analysis scripts.
% Run from MATLAB with:
%   run('simulink_q_dot/test/test_q_dot_analysis_scripts.m')
%
% Uses synthetic timeseries matching the current model's Diagnostics Outputs.
% It does not start ROS or simulate the robot.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
testDirectory = fullfile(repoRoot,'simulink_q_dot','test');
workspaceDirectory = tempname;
mkdir(workspaceDirectory);
cleanup = onCleanup(@() cleanupTestWorkspace(workspaceDirectory)); %#ok<NASGU>

originalDirectory = pwd;
restoreDirectory = onCleanup(@() cd(originalDirectory)); %#ok<NASGU>
addpath(testDirectory);
cd(workspaceDirectory);
assignin('base','resultsRoot',fullfile(workspaceDirectory,'test_results'));
assignin('base','showFigures',false);
assignin('base','closeFiguresAfterSave',true);
assignin('base','outputFolder',fullfile(workspaceDirectory,'test_results','joint_command_monitor'));

sampleTime = (0:0.01:2).';
sampleCount = numel(sampleTime);

% Current model Diagnostics Outputs (without the legacy log_ prefix).
assignin('base','image_error',timeseries([0.03*exp(-sampleTime),-0.02*exp(-sampleTime)],sampleTime));
assignin('base','ekf_state',timeseries([0.1+zeros(sampleCount,1),0.2+zeros(sampleCount,1),zeros(sampleCount,2)],sampleTime));
assignin('base','ekf_covariance',timeseries(repmat(reshape(eye(4),1,16),sampleCount,1),sampleTime));
assignin('base','v_target_hat',timeseries(zeros(sampleCount,2),sampleTime));
assignin('base','v_p',timeseries([0.01*ones(sampleCount,1),-0.01*ones(sampleCount,1)],sampleTime));
assignin('base','v_ff',timeseries(zeros(sampleCount,2),sampleTime));
assignin('base','v_issued',timeseries([0.01*ones(sampleCount,1),-0.01*ones(sampleCount,1)],sampleTime));
assignin('base','v_task_raw',timeseries([0.01*ones(sampleCount,1),-0.01*ones(sampleCount,1)],sampleTime));
assignin('base','innovation_nis',timeseries(0.5*ones(sampleCount,1),sampleTime));
assignin('base','target_data',timeseries([ones(sampleCount,1),1000+zeros(sampleCount,1),592+zeros(sampleCount,1),0.5*ones(sampleCount,1)],sampleTime));
assignin('base','target_is_new',timeseries(double(mod((0:sampleCount-1).',5)==0),sampleTime));
assignin('base','measurement_accepted',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','safe_valid',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','target_visible',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','ekf_initialized',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','control_active',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','controller_ok',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','kinematics_valid',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','camera_twist_measured',timeseries(zeros(sampleCount,6),sampleTime));
assignin('base','joint_position',timeseries(repmat([0,-0.7,0,-2.2,0,1.5,0.7],sampleCount,1),sampleTime));
assignin('base','joint_velocity',timeseries(zeros(sampleCount,7),sampleTime));
assignin('base','qdot_raw',timeseries(zeros(sampleCount,7),sampleTime));
assignin('base','qdot_command',timeseries(zeros(sampleCount,7),sampleTime));
assignin('base','velocity_scale',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','acceleration_scale',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','position_scale',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','guard_valid',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','target_age',timeseries(zeros(sampleCount,1),sampleTime));
assignin('base','joint_age',timeseries(zeros(sampleCount,1),sampleTime));
assignin('base','sigma_min',timeseries(0.30*exp(-sampleTime),sampleTime));
assignin('base','singularity_scale',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','damping_used',timeseries(0.015*ones(sampleCount,1),sampleTime));
assignin('base','solver_valid',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','control_active',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','kinematics_valid',timeseries(ones(sampleCount,1),sampleTime));
assignin('base','target_visible',timeseries(ones(sampleCount,1),sampleTime));

% Minimal parameter snapshot used by the analyzer and joint monitor.
assignin('base','Ts',0.01);
assignin('base','T_end',2);
assignin('base','fx',2057);
assignin('base','fy',2054);
assignin('base','cx',1000);
assignin('base','cy',592);
assignin('base','Z_hat',0.5);
assignin('base','rho_hat',2);
assignin('base','Kpx',1);
assignin('base','Kpy',1);
assignin('base','k_ff',0);
assignin('base','task_velocity_filter_tau',0.04);
assignin('base','enable_proportional',true);
assignin('base','enable_ekf_feedforward',false);
assignin('base','controller_enable',true);
assignin('base','P0',eye(4));
assignin('base','Q_ekf',eye(4));
assignin('base','R_ekf',eye(2));
assignin('base','ekf_gate_threshold',5.991);
assignin('base','singularity_sigma_stop',0.025);
assignin('base','singularity_sigma_warn',0.080);
assignin('base','required_valid_frames',5);
assignin('base','measurement_reset_timeout',2);
assignin('base','target_timeout_sec',0.5);
assignin('base','joint_state_timeout_sec',0.5);
assignin('base','camera_velocity_feedback_timeout_sec',0.5);
assignin('base','joint_position_min',[-2.3476;-1.5454;-2.4937;-2.7714;-2.5100;0.7773;-2.7045]);
assignin('base','joint_position_max',[2.3476;1.5454;2.4937;-0.4226;2.5100;4.2841;2.7045]);
assignin('base','joint_velocity_limit',0.2*ones(7,1));
assignin('base','joint_acceleration_limit',1.5*ones(7,1));

assignin('base','testName','synthetic_current_model');
assignin('base','testType','closed_loop');
evalin('base',sprintf('run(''%s'')',fullfile(testDirectory,'save_single_camera_test_results.m')));

resultFolders = dir(fullfile(workspaceDirectory,'test_results','*closed_loop_synthetic_current_model'));
assert(~isempty(resultFolders),'Primary analyzer did not create a run folder.');
runFolder = fullfile(resultFolders(1).folder,resultFolders(1).name);
assert(isfile(fullfile(runFolder,'summary.txt')),'Primary analyzer did not write summary.txt.');
assert(isfile(fullfile(runFolder,'selected_logs_parameters_metrics.mat')),'Primary analyzer did not save selected logs and metrics.');
summaryText = fileread(fullfile(runFolder,'summary.txt'));
assert(contains(summaryText,'ekf_state'),'Current model EKF state was not mapped into the analysis.');
assert(contains(summaryText,'qdot_command'),'Current model joint command was not captured in the analysis.');

% The compatibility entry point should invoke the same current analyzer.
assignin('base','testName','synthetic_compatibility');
evalin('base',sprintf('run(''%s'')',fullfile(testDirectory,'save_single_camera_test_results_2.m')));
compatibilityFolders = dir(fullfile(workspaceDirectory,'test_results','*synthetic_compatibility'));
assert(~isempty(compatibilityFolders),'Compatibility script did not create analyzer output.');

% Joint monitor should accept current-model names and derive acceleration if needed.
evalin('base',sprintf('run(''%s'')',fullfile(testDirectory,'plot_fr3_joint_limit_test.m')));
monitorFiles = dir(fullfile(workspaceDirectory,'test_results','joint_command_monitor','*.csv'));
assert(numel(monitorFiles)==3,'Joint monitor did not write all three metric CSV files.');

fprintf('PASS: current-model diagnostic aliases, compatibility entry point, and joint monitor.\n');

function cleanupTestWorkspace(workspaceDirectory)
if isfolder(workspaceDirectory)
    rmdir(workspaceDirectory,'s');
end
end

function report = test_q_dot_ros2_buses(modelFile)
%TEST_Q_DOT_ROS2_BUSES Check Subscriber seed types and simulate without publishers.
% Run with single_camera_q_dot closed. This test never saves the loaded model.
% All ROS 2 Publish blocks are removed in memory before update or simulation.

if nargin == 0
    testFolder = fileparts(mfilename('fullpath'));
    modelFile = fullfile(testFolder, '..', 'single_vision_q_dot', ...
        'single_camera_q_dot.slx');
end
assert(isfile(modelFile), 'qdot:MissingModel', 'Model not found: %s', modelFile);
[~, modelName] = fileparts(modelFile);
assert(~bdIsLoaded(modelName), 'qdot:ModelAlreadyOpen', ...
    'Save and close %s before running this test.', modelName);
load_system(modelFile);
closeWithoutSaving = onCleanup(@() close_system(modelName, 0)); %#ok<NASGU>

publishers = find_system(modelName, 'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', 'ReferenceBlock', 'ros2lib/Publish');
for index = 1:numel(publishers)
    publisherSubsystem = get_param(publishers{index}, 'Parent');
    assert(~strcmp(publisherSubsystem, modelName), ...
        'qdot:UnexpectedPublisher', 'Publisher must be inside a subsystem.');
    if getSimulinkBlockHandle(publisherSubsystem) ~= -1
        delete_block(publisherSubsystem);
    end
end
assert(isempty(find_system(modelName, 'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', 'ReferenceBlock', 'ros2lib/Publish')), ...
    'qdot:PublishersPresent', 'Refusing to test a model with command publishers.');

set_param(modelName, 'SimulationCommand', 'update');
subscribers = find_system(modelName, 'FollowLinks', 'on', ...
    'LookUnderMasks', 'all', 'ReferenceBlock', 'ros2lib/Subscribe');
assert(numel(subscribers) == 2, 'qdot:SubscriberCount', ...
    'Expected the target and joint-state Subscribers.');

mismatches = {};
report = struct('model', modelFile, 'publishersRemoved', numel(publishers), ...
    'subscribersChecked', numel(subscribers), 'simulationStopTime', 0.05);
for index = 1:numel(subscribers)
    subscriber = subscribers{index};
    messageType = get_param(subscriber, 'messageType');
    [expectedType, expectedBus] = ...
        ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(messageType);
    actualType = get_param([subscriber '/Constant'], 'OutDataTypeStr');
    source = [subscriber '/SourceBlock'];
    if ~strcmp(actualType, expectedType) || ...
            ~strcmp(get_param(source, 'SLBusName'), expectedBus) || ...
            ~strcmp(get_param(source, 'ROSMessageType'), messageType)
        mismatches{end + 1} = sprintf('%s: seed=%s; expected=%s', ...
            subscriber, actualType, expectedType); %#ok<AGROW>
    end
end
assert(isempty(mismatches), 'qdot:SubscriberBusMismatch', '%s', ...
    strjoin(mismatches, newline));

for index = 1:numel(subscribers)
    messageType = get_param(subscribers{index}, 'messageType');
    converter = ros.slros2.internal.sim.ROSMsgToBusStructConverter( ...
        messageType, modelName);
    seed = converter.convert(ros2message(messageType));
    bus = ros.slros2.internal.bus.Util.getBusObjectFromMsgType( ...
        messageType, modelName);
    assert(isequal(fieldnames(seed), {bus.Elements.Name}'), ...
        'qdot:ConvertedBusMismatch', 'Converted fields differ for %s.', messageType);
end

sim(modelName, 'StopTime', num2str(report.simulationStopTime), ...
    'ReturnWorkspaceOutputs', 'on');
fprintf('PASS: %d Subscriber seed types and converted fields; %d publishers removed.\n', ...
    report.subscribersChecked, report.publishersRemoved);
end

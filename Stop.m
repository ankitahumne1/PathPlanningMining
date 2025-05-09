modelName = 'PathFollower';
modelName1 = 'Vehicle';
hostModel = 'Host';
tg = slrealtime('TargetPC1');
tg1 = slrealtime('TargetPC2');
tg.stop;
tg1.stop;

set_param(hostModel, 'SimulationCommand', 'stop');

myTimers = timerfind;

if ~isempty(myTimers)
    stop(myTimers);
    delete(myTimers);
end
clear all
disp('Execution stopped');
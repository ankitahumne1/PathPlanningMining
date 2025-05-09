clear all;

modelName = 'PathFollower';
modelName1 = 'Vehicle';
hostModel = 'Host';

% myTimer = timerInBackground;
ipAddress = helperIPAddresses;
controllerParams =  helperPathFollowerParams;
vehicleParams = helperVehicleParams;
hostParams =  helperScenarioParams;
load_system(modelName);
load_system(modelName1);
load_system(hostModel);

tg = slrealtime('TargetPC1');
tg1 = slrealtime('TargetPC2');
% tg.configureModelForTargetPlatform(modelName)
tg.connect;
slbuild(modelName);
tg.load(modelName);
tg1.connect;
slbuild(modelName1);
tg1.load(modelName1);
% try
%     tg.connect;
% catch
%     tg.update;
%     tg.connect;
% end
% 
% try
%     tg.stop;
%     tg.load(modelName);
% catch
%     slbuild(modelName);
%     tg.load(modelName);
% end



% tg = slrealtime('TargetPC2');
% try
%     tg1.connect;
% catch
%     tg1.update;
%     tg1.connect;
% end
% 
% try
%     tg1.stop;
%     tg1.load(modelName1);
% catch
%     slbuild(modelName1);
%     tg1.load(modelName1);
% end

disp('Starting Execution');
myTimer = timerInBackground;


try
    start(myTimer);
catch
    stop(myTimer);
    pause(1);
    start(myTimer);
end
% set_param(hostModel, 'SimulationCommand', 'start');
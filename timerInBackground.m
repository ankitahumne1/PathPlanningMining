function [myTimer] = timerInBackground
    myTimer = timer('Name','MyTimer',               ...
                    'Period',20,                 ... 
                    'StartDelay',5,                 ... 
                    'TasksToExecute',inf,           ... 
                    'ExecutionMode','fixedSpacing', ...
                    'TimerFcn',@myTimerCallback); 
    start(myTimer);
end

function myTimerCallback(hObject, eventdata)    
    try
        tg = slrealtime('TargetPC1');
        tg.connect;
        tg1 = slrealtime('TargetPC2');
        tg1.connect;
        start(tg,'ReloadOnStop',true);
        start(tg1,'ReloadOnStop',true);
        set_param("Host", 'SimulationCommand', 'start');
        
    catch
        disp('Reconnecting in 20s.')
    end
end
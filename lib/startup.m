function figure(hObject, eventdata, handles)
    addpath('/lib');

    S = [0 0 1 0 0 0;
         0 -1 0 -cross([0 -1 0], [0 0 0.089459]);
         0 -1 0 -cross([0 -1 0], [-0.425 0 0.089459]);
         0 -1 0 -cross([0 -1 0], [-0.8173 0 0.089459]);
         0 0 -1 -cross([0 0 -1], [-0.8173 -0.10915 0]);
         0 -1 0 -cross([0 -1 0], [-0.8173 0 -0.0052])]';
    
    % Home configuration
    M = [1	0	0	-0.817;
        0	0	-1	-0.19;
        0	1	0	-0.005;
        0	0	0	1];
    n = 6;
    g = [0 0 -9.81];

    currentQ = [0 0 0 0 0 0];
    currentPose = MatrixLog6(fkine(S, M, currentQ, 'space'));
    currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';
    
    mdl_ur5;
    robot = ur5;
    robot.plot(zeros(1,6))
    
    [Mlist, Glist] = make_dynamics_model(robot);
    
    Ftip = [0 0 0 0 0 0];

    guidata(hObject, handles)
end

function x(hObject, eventdata, handles)
    handles.X = get(hObject, 'String')
end
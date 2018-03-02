

[success, grid_state] = runBicycleTest(0,0,3,0,0,2,0,[23,16,-2], 0,0);
[success, lqr_state] = runBicycleTest(0,0,3,0,0,2,0,[11.1,2.4,-6.6], 0,0);
[success, control_state] = runBicycleTest(0,0,3,0,0,2,0,[71,21,-20], 0,0);

figure('Name','Controller comparison with intial lean rate = 2')
subplot(2,2,1)

    times = grid_state(1:200,1);
    phi = grid_state(1:200,4);
    delta = grid_state(1:200,6);
    phidot = grid_state(1:200,7);
    deltadot = diff(delta)./diff(times);

    subplot(2,2,1)
    plot(times,phi);
    hold on
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
    subplot(2,2,2)
    hold on
    plot(times,phidot);
    title('lean rate vs. time');
    xlabel('time (s)');
    ylabel('phi-dot');
    subplot(2,2,3)
    hold on
    plot(times,delta);
    title('steer vs. time');
    xlabel('time (s)');
    ylabel('delta');
    subplot(2,2,4)
    hold on
    plot(times(1:end-1),deltadot);
    title('steer rate vs. time');
    xlabel('time (s)');
    ylabel('deltadot');
    
    
    times = lqr_state(1:200,1);
    phi = lqr_state(1:200,4);
    delta = lqr_state(1:200,6);
    phidot = lqr_state(1:200,7);
    deltadot = diff(delta)./diff(times);

    hold on
    subplot(2,2,1)
    plot(times,phi);
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
    subplot(2,2,2)
    plot(times,phidot);
    title('lean rate vs. time');
    xlabel('time (s)');
    ylabel('phi-dot');
    subplot(2,2,3)
    plot(times,delta);
    title('steer vs. time');
    xlabel('time (s)');
    ylabel('delta');
    subplot(2,2,4)
    plot(times(1:end-1),deltadot);
    title('steer rate vs. time');
    xlabel('time (s)');
    ylabel('deltadot');
   
    
    times = control_state(1:200,1);
    phi = control_state(1:200,4);
    delta = control_state(1:200,6);
    phidot = control_state(1:200,7);
    deltadot = diff(delta)./diff(times);

    hold on
    subplot(2,2,1)
    plot(times,phi);
    title('lean vs. time');
    xlabel('time (s)');
    ylabel('phi');
    subplot(2,2,2)
    plot(times,phidot);
    title('lean rate vs. time');
    xlabel('time (s)');
    ylabel('phi-dot');
    subplot(2,2,3)
    plot(times,delta);
    title('steer vs. time');
    xlabel('time (s)');
    ylabel('delta');
    subplot(2,2,4)
    plot(times(1:end-1),deltadot);
    title('steer rate vs. time');
    xlabel('time (s)');
    ylabel('deltadot');
    


   legend("grid search","lqr","control")


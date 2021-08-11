% Get Robotarium object used to communicate with the robots/simulator
%%%%% Do not touch this section!!!
rb = RobotariumBuilder();
%%%%%

% Agent parameters (swarm size, blind spot,
% repulsion-orientation-attraction, current sensor estimates, storage for 
% sensor estimates,  how much to weigh neighbors estimates, the number of 
% nearest neighbors to consider if using the topological comms model, the 
% total iterations of the simulation, the id of the compromised agent
% Particular radii values used here are in terms of body length (0.08)
N = 10;
agent_memory = zeros(N,N);
half_sense = pi*105/180;
rr = 0.16; %0.08 radius of each robot
ro = 0.24;
ra = 4.00;
sensor_est = [1:1:N]'; %initial estimate is id; robot i initially estimates i
sensor_est_history = []; %store sensor estimates
weight_neighbors = 0.001;
top_dist = 4;
total_iterations = 2500;
% Compromised agent is chosen at random. This agent does not run the
% agreement protocol with other agents, but instead, keeps broadcasting a
% value where it drives the estimates of the swarm
compromised_agent = randi(N);
malicious_estimate = compromised_agent;

%%%%% Do not touch this section!!!
r = rb.set_number_of_agents(N).set_save_data(false).build();
x = r.get_poses();
r.step();
si_barrier_certificate = create_si_barrier_certificate('SafetyRadius',0.001);
si_to_uni_dynamics = create_si_to_uni_mapping2();
args = {'PositionError', 0.01, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();
dxi = zeros(2,N); % The 2D velocity vector of all N robots
%%%%%

% For each iteration in time...
for t = 1:1:total_iterations
    
    % Get the latest pose information
    x = r.get_poses();
    
    % Characterize what the group as a whole is doing
    [Op,Or] = get_params(x, N);
    
    % Check everyone's blind spots
    [blind_neighbors] = check_blind_spot(x, N, half_sense);
    
    % Get everyone's 2D velocity vector
    [dxi] = swarm(rr, ro, ra, x, blind_neighbors, N, dxi);
    
    % Modify everyone's 2D velocity vector in order to stay in the arena
    dxi = bounce_off_wall(dxi, x, N);
    
    % Get comms neighbors using the metric model and setting the metric
    % range to the radius of attraction
    [neighbors] = find_neighbors('M', N, x, top_dist, ra);

    % Update and store sensor estimate
    % if agent is transmitted the same value twice in a row, the agent no
    % longer accepts information from the other agent
    % if the ignored agent goes away and comes back it will accept values
    % again until the next duplicate case
    [delta_sensor_est, agent_memory] = estimate(sensor_est, neighbors, N, agent_memory); %change in estimate 
    sensor_est = sensor_est + (weight_neighbors * delta_sensor_est); %incorporate that change
    sensor_est(compromised_agent, 1) = malicious_estimate; %malicious broadcast
    sensor_est_history = [sensor_est_history, sensor_est];
    
    %%%%% Do not touch this section!!!
    % Robotarium uses dxi to move the agents
    dxi = si_barrier_certificate(dxi, x(1:2, :));
    dxu = si_to_uni_dynamics(dxi, x);
    r.set_velocities(1:N, dxu);
    r.step();
    %%%%%
    
    % Save a frame every 100 iterations
    if (~mod(t,100))
        saveas(gcf,['t',num2str(t)],'epsc');
    end
    
end

%%%%% Do not touch this section!!!
r.call_at_scripts_end();
figure; % Plot the sensor estimates for each agent over time
for ii=1:1:N
    plot(sensor_est_history(ii, 1:1:total_iterations), 'LineWidth', 2);
    hold on;
end
axis([-100 total_iterations 0 10]);
xlabel('time iteration');
ylabel('sensor estimate');
saveas(gcf,'sensor_estimation','epsc');
%%%%%




struct EndOfSimException <: Exception end
struct EndOfVehicleException <: Exception end

function vehicle_simulate(state::MechanismState{X}, mviss, vehicle_id, final_time, internal_control!, external_control!, publisher!; Δt=1e-4, stabilization_gains=RigidBodyDynamics.default_constraint_stabilization_gains(X), max_realtime_rate=Inf) where X
    T = RigidBodyDynamics.cache_eltype(state)
    result = DynamicsResult{T}(state.mechanism)
    control_torques = similar(velocity(state))
    external_wrenches = Dict{RigidBodyDynamics.BodyID, RigidBodyDynamics.Wrench{T}}()
    closed_loop_dynamics! = let result=result, control_torques=control_torques, stabilization_gains=stabilization_gains 
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)
            internal_control!(control_torques, t, state)
            external_control!(external_wrenches, t, state)
            publisher!(t, state)
            dynamics!(result, state, control_torques, external_wrenches; stabilization_gains=stabilization_gains)
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end
    tableau = RigidBodyDynamics.runge_kutta_4(T)
    sink = FollowCamSink(mviss, vehicle_id)
    integrator = RigidBodyDynamics.MuntheKaasIntegrator(state, closed_loop_dynamics!, tableau, sink)
    RigidBodyDynamics.integrate(integrator, final_time, Δt; max_realtime_rate)
end

function vis_updater(mvisualizers, channels;
                     follow_dist=35.0, 
                     follow_height=6.0,
                     follow_offset=6.0)
    while true
        for (id, channel) in channels
            if isready(channel)
                vehicle_state = take!(channel)
                if vehicle_state.persist
                    for (e, mvis) in enumerate(mvisualizers)
                        set_configuration!(mvis, configuration(state))
                        if e == id
                            config = configuration(state)
                            quat = config[1:4]
                            pose = config[5:7]
                            yaw = extract_yaw_from_quaternion(quat) 
                            offset = [sink.follow_dist * [cos(yaw), sin(yaw)]; -sink.follow_height] + sink.follow_offset * [sin(yaw), -cos(yaw), 0]
                            setcameratarget!(sink.vis[sink.follow_cam_id].visualizer, pose)
                            setcameraposition!(sink.vis[sink.follow_cam_id].visualizer, pose-offset)
                        end
                    end
                else
                    foreach(mvis->delete_car(mvis), mvisualizers)
                end
            end
        end
    end
end

function load_mechanism()
    urdf_path = joinpath(dirname(pathof(VehicleSim)), "assets", "chevy.urdf")
    chevy_base = parse_urdf(urdf_path, floating=true)
    chevy_joints = joints(chevy_base)
    (; urdf_path, chevy_base, chevy_joints)
end

function server(max_vehicles=1, port=4444; full_state=true, rng=MersenneTwister(1))
    host = getipaddr()
    map = training_map()
    server_visualizer = get_vis(map, true, host)
    @info "Server can be connected to at $host and port $port"
    @info inform_hostport(server_visualizer, "Server visualizer")
    client_visualizers = [get_vis(map, false, host) for _ in 1:max_vehicles]
    all_visualizers = [client_visualizers; server_visualizer]

    (; urdf_path, chevy_base, chevy_joints) = load_mechanism()
    chevy_visuals = URDFVisuals(urdf_path, package_path=[dirname(pathof(VehicleSim))])

    viable_segments = Set(keys(map))
    vehicles = Dict()
    for vehicle_id in 1:max_vehicles
        local seg
        while true
            seg_id = rand(rng, viable_segments)
            delete!(viable_segments, seg_id)
            if contains_lane_type(map[seg_id], intersection, stop_sign)
                continue
            else
                seg = map[seg_id]
                break
            end 
        end
        vehicle = spawn_car_on_map(all_visualizers, seg, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
        vehicles[vehicle_id] = vehicle
    end

    #vehicle_count = 0
    #sim_task = errormonitor(@async begin
    #    server = listen(host, port)
    #    while true
    #        try
    #            sock = accept(server)
    #            vehicle_count += 1
    #            msg = inform_hostport(client_visualizers[vehicle_count], "Client follow-cam")
    #            @info "Client accepted."
    #            # open(client_visualizers[vehicle_count])
    #            serialize(sock, msg)
    #            errormonitor(@async spawn_car(all_visualizers, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_count, server; full_state))
    #        catch e
    #            break
    #        end
    #    end
    #end)
end

function spawn_car_on_map(visualizers, seg, chevy_base, chevy_visuals, chevy_joints, vehicle_id)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    configure_contact_points!(chevy)
    state = MechanismState(chevy)

    mviss = map(visualizers) do vis
        MechanismVisualizer(chevy, chevy_visuals, vis)
    end
 
    config = get_initialization_point(seg)
    foreach(mvis->configure_car!(mvis, state, joints(chevy), config), mviss)
    (; chevy, state, mviss)
end

function spawn_car(visualizers, sock, chevy_base, chevy_visuals, chevy_joints, vehicle_id, server; full_state=false)
    chevy = deepcopy(chevy_base)
    chevy.graph.vertices[2].name="chevy_$vehicle_id"
    configure_contact_points!(chevy)
    state = MechanismState(chevy)

    mviss = map(visualizers) do vis
        MechanismVisualizer(chevy, chevy_visuals, vis)
    end

    config = CarConfig(SVector(-7,12,2.5), 0.0, 0.0, 0.0, 0.0)
    foreach(mvis->configure_car!(mvis, state, joints(chevy), config), mviss)
    
    v = 0.0
    θ = 0.0
    state_q = state.q
    state_v = state.v
    persist = true
    shutdown = false
    set_reference! = (cmd) -> begin
        if !cmd.persist 
            @info "Destroying vehicle."
            close(sock)
        end
        if cmd.shutdown
            @info "Shutting down server."
            close(sock)
        end
            
        v = cmd.velocity # rename
        θ = cmd.steering_angle
        shutdown=cmd.shutdown
        persist=cmd.persist
    end
    control! = (torques, t, state) -> begin
            !persist && throw(EndOfVehicleException())
            shutdown && throw(EndOfSimException())
            torques .= 0.0
            steering_control!(torques, t, state; reference_angle=θ)
            suspension_control!(torques, t, state)
            nothing
    end
    wrenches! = (bodyid_to_wrench, t, state) -> begin
        RigidBodyDynamics.update_transforms!(state)
        wheel_control!(bodyid_to_wrench, chevy, t, state; reference_velocity=v)
    end
    publisher! = (t, state) -> begin
        state_q .= state.q
        state_v .= state.v
    end

    @async while isopen(sock)
        car_cmd = deserialize(sock)
        set_reference!(car_cmd)
    end 
 
    @async while isopen(sock)
        ground_truth_msg = GroundTruthMeasurement(state_q, state_v)
        meas_msg = MeasurementMessage([ground_truth_msg,])
        serialize(sock, meas_msg)
    end

    try 
        vehicle_simulate(state, 
                         mviss,
                         vehicle_id,
                         Inf, 
                         control!, 
                         wrenches!,
                         publisher!;
                         max_realtime_rate=1.0)
    catch e
        foreach(mvis->delete_vehicle!(mvis), mviss)
        if e isa EndOfSimException
            close(server)
        end
    end
end


struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    controlled::Bool
end

isfull(ch::Channel) = begin
    if ch.sz_max===0
        isready(ch)
    else
        length(ch.data) ≥ ch.sz_max
    end
end

function get_c()
    ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
    ret == 0 || error("unable to switch to raw mode")
    c = read(stdin, Char)
    ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    c
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)
    msg = deserialize(socket) # Visualization info
    @info msg

    @async while isopen(socket)
        sleep(0.001)
        state_msg = deserialize(socket)
        measurements = state_msg.measurements
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0
        for meas in measurements
            if meas isa GroundTruthMeasurement
                num_gt += 1
            elseif meas isa CameraMeasurement
                num_cam += 1
            elseif meas isa IMUMeasurement
                num_imu += 1
            elseif meas isa GPSMeasurement
                num_gps += 1
            end
        end
        @info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
    end
    
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    @info "Press 'q' at any time to terminate vehicle."
    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        cmd = VehicleCommand(steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
end



function auto_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    (peer_host, peer_port) = getpeername(socket)

    msg = deserialize(socket) # Visualization info
    @info msg

    #map_segments = training_map()
    quit_channel = Channel{Bool}(1)

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    #perception_state_channel = Channel{MyPerceptionType}(1)

    #target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)
    put!(quit_channel, false)
    @info "Press 'q' at any time to terminate vehicle."

    routes::Vector{Int} = route(38,32)
    midpoint_paths::Vector{MidPath} = midpoints(routes)

    @info "Measurement populator thread entering intialization"
    errormonitor(@async while !fetch(quit_channel) && isopen(socket)
        sleep(0.001)
        

        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        
        num_cam = 0
        num_imu = 0
        num_gps = 0
        num_gt = 0

        !received && continue
        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id

        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            end
        end
    end)

    @async fake_localize(gt_channel, localization_state_channel, ego_vehicle_id,quit_channel)
    @async path_planning(socket,quit_channel, localization_state_channel, routes, midpoint_paths)

    while !fetch(quit_channel) && isopen(socket)
        @info "Key Catcher Loop entered"
        key = get_c()
        if key == 'q'
            # terminate vehicle
            take!(quit_channel)
            put!(quit_channel, true)         
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Auto Client."
        end
    end 
    #=  ######## Add in ltr ############### 
    #### Zygote pkg autodiff, symbolic diff, forwdiff ####
    @async while fetch(quit_channel) && isopen(socket)
        localize(gps_channel, imu_channel, localization_state_channel)
    end 
    @async while fetch(quit_channel) && isopen(socket) 
        perception(cam_channel, localization_state_channel, perception_state_channel)
    end 
    
    #diegsters algorithm find shortest path in cyclic directed graph (edmonds algo)  import GraphFlows

    =#
end

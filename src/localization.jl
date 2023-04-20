
function convert_to_localization_type(gt_meas)
    time = gt_meas.time
    pos = gt_meas.position
    orien = gt_meas.orientation
    vel = gt_meas.velocity
    ang_vel = gt_meas.velocity
    size = gt_meas.size

    vehicle_state = FullVehicleState(pos, orien, vel, ang_vel)

    MyLocalizationType(time , vehicle_state, size )
end

function fake_localize(
    gt_channel::Channel{GroundTruthMeasurement}, 
    localization_state_channel::Channel{MyLocalizationType},
    ego_id, 
    quit_channel)
    quit_msg= fetch(quit_channel)

    while !fetch(quit_channel)
        sleep(0.01)

        #@info "Fake Localization loop entrred"
        fresh_gt_meas = []
        wait = 0


        #@info "waiting for gt channel to have data"
        try 
            wait(gt_channel)  
        catch e
            #@info "wait err: $e"
        end
        

        while length(gt_channel.data) > 0
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        if length(fresh_gt_meas) == 0 
            continue
        end 

        latest_meas_time = fresh_gt_meas[1].time 
        latest_meas = fresh_gt_meas[1]
        for meas in fresh_gt_meas
            if meas.time > latest_meas_time && meas.vehicle_id == ego_id
                latest_meas = meas
                latest_meas_time = meas.time
            end
        end

        # Convert latest_meas to MyLocalizationType
        my_converted_gt_message = convert_to_localization_type(latest_meas)


        if isready(localization_state_channel)
            take!(localization_state_channel)
        end

        put!(localization_state_channel,my_converted_gt_message) 

        state_channel = fetch(localization_state_channel)
        #@info "Fake localization channel: $state_channel "
    end
end


function localize(gps_channel, imu_channel, localization_state_channel, quit_channel)
    @info "Starting localization"

    # initial state of vehicle
    x0 = zeros(13)
    first_gps = take!(gps_channel)
    @info "First GPS: $first_gps"

    # cur_seg = get_cur_segment([first_gps.lat, first_gps.long])
    # @info "Current segment: $cur_seg"

    θ = first_gps.heading

    # rotation matrix from segment to world frame
    R = RotZ(θ)

    # get quaternion from rotation matrix
    qw = sqrt(1 + R[1,1] + R[2,2] + R[3,3]) / 2
    qx = (R[3,2] - R[2,3]) / (4 * qw)
    qy = (R[1,3] - R[3,1]) / (4 * qw)
    qz = (R[2,1] - R[1,2]) / (4 * qw)
    x0[4:7] = [qw, qx, qy, qz]

    x0[1:2] = [first_gps.lat, first_gps.long]
    x0[3] = 1.0 
    x = x0
    last_update = 0.0

    P = zeros(13, 13)
    diag_vals = [
        1.0, 1.0, 1.0, 
        0.1, 0.1, 0.1, 0.1, 
        0.1, 0.1, 0.1, 
        0.1, 0.1, 0.1
    ]
    P = diagm(diag_vals)

    # process noise
    Q = 0.1 * I(13)

    # measurement noise for both GPS and IMU
    R_gps = Diagonal([3.0, 3.0, 1.0])
    R_imu = 0.1 * I(6) # 0.01?

    @info "Starting localization loop"

    # Set up algorithm / initialize variables
    while !fetch(quit_channel) 
        sleep(0.001) # prevent thread from hogging resources & freezing other threads
    
        fresh_gps_meas = []
        #@info "b4 data"
        while isready(gps_channel)
            meas = take!(gps_channel)
            push!(fresh_gps_meas, meas)
        end
        fresh_imu_meas = []
        while isready(imu_channel)
            meas = take!(imu_channel)
            push!(fresh_imu_meas, meas)
        end

        #@info "after data"
        # process measurements
        while length(fresh_gps_meas) > 0 && length(fresh_imu_meas) > 0
            sleep(0.001)
            # grab measurements and sort them by time
            all_meas = [fresh_gps_meas..., fresh_imu_meas...]
            sort!(all_meas, by = m -> m.time)

            # Process the earliest measurement
            z = all_meas[1]
            Δ = z.time - last_update
            last_update = z.time

            # Remove the processed measurement from the respective list
            if z isa GPSMeasurement
                fresh_gps_meas = fresh_gps_meas[2:end]
            elseif z isa IMUMeasurement
                fresh_imu_meas = fresh_imu_meas[2:end]
            end

            #@info "Processing measurement of type $(typeof(z))"

            # run filter
            if z isa GPSMeasurement
                R = R_gps
            elseif z isa IMUMeasurement
                R = R_imu
            end
            x, P = filter(x, z, P, Q, R, Δ)

            # publish state
            full = FullVehicleState(x[1:3], x[4:7], x[8:10], x[11:13])
            localization_state = MyLocalizationType(last_update, full, [13.2, 5.7, 5.3])
            if isready(localization_state_channel)
                take!(localization_state_channel)
            end
            put!(localization_state_channel, localization_state)
            #@info "localization populated"
        end
    end
end


function custom_roty(θ)
    R = zeros(3, 3)
    R = [cos(θ) 0 sin(θ); 0 1 0; -sin(θ) 0 cos(θ)]
    return R
end

function get_imu_transform1()
    R_imu_to_body = custom_roty(0.02)
    t_imu_to_body = [0, 0, 0.7]

    T = [R_imu_to_body t_imu_to_body]
end


# -------------------------------- EKF functions -------------------------------- #
# process model
function f1(x, Δt)
    position = x[1:3]
    quaternion = x[4:7]
    velocity = x[8:10]
    angular_vel = x[11:13]

    r = angular_vel
    mag = norm(r)

    if mag < 1e-5
        sᵣ = 1.0
        vᵣ = zeros(3)
    else
        sᵣ = cos(mag*Δt / 2.0)
        vᵣ = sin(mag*Δt / 2.0) * (r / mag)
    end

    sₙ = quaternion[1]
    vₙ = quaternion[2:4]

    s = sₙ*sᵣ - vₙ'*vᵣ
    v = sₙ*vᵣ+sᵣ*vₙ+vₙ×vᵣ

    R = Rot_from_quat(quaternion)  

    new_position = position + Δt * R * velocity
    new_quaternion = [s; v]
    new_velocity = velocity
    new_angular_vel = angular_vel
    return [new_position; new_quaternion; new_velocity; new_angular_vel]
end

function jac_fx(x, Δ)
    # take the gradient of f with respect to x
    jacobian(x -> f(x, Δ), x)[1]
end

# measurement model
function h(x, z)
    if z isa GPSMeasurement
        T = get_gps_transform()
        gps_loc_body = T*[zeros(3); 1.0]
        xyz_body = x[1:3] # position
        q_body = x[4:7] # quaternion

        Tbody = get_body_transform(q_body, xyz_body)
        xyz_gps = Tbody * [gps_loc_body; 1]
        yaw = extract_yaw_from_quaternion(q_body)
        meas = [xyz_gps[1:2]; yaw]

        return meas
    elseif z isa IMUMeasurement
        # convert to body frame
        T_body_imu = get_imu_transform1()
        T_imu_body = invert_transform(T_body_imu)
        R = T_imu_body[1:3,1:3]
        p = T_imu_body[1:3,end]

        v_body = x[8:10]
        ω_body = x[11:13]

        ω_imu = R * ω_body
        v_imu = R * v_body + p × ω_imu

        # need to return an imu frame, which is the linear and angular velocity of the body frame
        return [v_imu; ω_imu]
    else
        error("Unknown measurement type")
    end
end

function jac_hx(x, z)
    # take the gradient of h with respect to x
    jacobian(x -> h(x, z), x)[1]
end

# convert z to a vector
function z_to_vec(z)
    if z isa GPSMeasurement
        return [z.lat; z.long; z.heading]
    elseif z isa IMUMeasurement
        return [z.linear_vel; z.angular_vel]
    else
        error("Unknown measurement type")
    end
end


function filter(x, z, P, Q, R, Δ)
    # predict
    x̂ = f1(x, Δ)
    F = jac_fx(x, Δ)
    P̂ = F * P * F' + Q

    # update
    z_vec = z_to_vec(z)

    y = z_vec - h(x̂, z)
    H = jac_hx(x̂, z)
    S = H * P̂ * H' + R
    K = P̂ * H' * inv(S)

    x̂ = x̂ + K * y
    P = (I - K * H) * P̂
    #@info "x: $x"

    return x̂, P
end
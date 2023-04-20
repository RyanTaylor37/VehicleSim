function perception(cam_meas_channel, localization_state_channel, perception_state_channel)

    @info "Starting perception loop"
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        # process bounding boxes / run ekf / do what you think is good

        # <-------- setup and run ekf -------->

        # initial state of other vehicle [p1, p2, theta, velocity, length, width, height]
        x0 = [latest_localization_state[1] + 5, latest_localization_state[2] + 5, 0, latest_localization_state[4], 8, 5, 5]

        # covariance for process model
        covariance_process = Diagonal([
            100.0, 100.0, # position (xy)
            80.0, # orientation
            80.0, # velocity
            0.0, 0.0, 0.0 # length, width, and height
        ])

        # covariance for measurements
        meas_covariance = Diagonal([
            80.0, 80.0, 80.0, 80.0 # y1, y2, (top-left corner) y3, y4 (bottom-right corner)
        ])

        
        # mean values for the state of the other car 
        μ=zeros(7)

        # covariance for the mean values
        Σ = Diagonal([
            100.0, 100.0, # position (xy)
            80.0, # orientation
            80.0, # velocity
            0.0, 0.0, 0.0 # length, width, and height
        ])

        # variables to be updated at each time setup
        μs = [μ,] # means
        Σs = Matrix{Float64}[Σ,] #covariances
        zs = Vector{Float64}[] #measurements
        timesteps = [] # timestep

        x_prev = x0

        num_steps = 25
        for k = 1:num_steps
            # QUESTION: what should we use as the timestep?
            xₖ = f(x_prev, 5)
            x_prev = xₖ

            zₖ = h(xₖ, perception_state_channel.focal_length, perception_state_channel.image_width, perception_state_channel.image_height)

            A = jac_fx(μs[k], 5)
            μ̂  = f(μs[k], 5)
            Σ̂  = Σ + A * Σs[k] * A'

            # <-------- process bounding boxes -------->


            # update variables
            # only taking the first element of the top row for right and left
            # only taking first element from the bottom row for top and bottom
            C = [top[2, :]; left[1, :]; bottom[2, :]; right[1, :]]
            Σₖ = inv(inv(Σ̂ ) + C' * inv(meas_covariance) * C)
            μₖ = Σₖ * ( inv(Σ̂ ) * μ̂ + C' * inv(meas_covariance) * zₖ)
            push!(μs, μₖ)
            push!(Σs, Σₖ)
            push!(zs, zₖ)

            perception_state = μs[i]
            put!(perception_state_channel, perception_state)


        end

    end

end

# returns top, left, right, and bottom extents of bounding boxes
function h(x, focal_length, image_width, image_height)
    
    # get the 8 corners of the other car in world frame
    # How do I do this without havine the quaternion?
    corners[1:8] = get_3d_bbox_corners(x, [13.2, 5.7, 5.3])

    # will hold the corners in camera frame
    corners_in_camera = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

    # will hold the pixel coordinates of each corner
    pixel_coordinates = [[0 ; 0], [0 ; 0], [0 ; 0], [0 ; 0], [0 ; 0], [0 ; 0] , [0 ; 0], [0 ; 0]]

    for k = 1:8

        # transform the points from world frame into camera frame
        corners_in_camera[i] = get_cam_transform(fresh_cam_meas[1].camera_id) * [corners[i]; 1]

    end

    # keeps track of which pixel coordinates is being updated
    iteration = 1

    # camera projection
    for corner in corners_in_camera

        # find the pixel values 
        pixel_y1 = focal_length * corner[1] / corner[3]
        pixel_y2 = focal_length * corner[2] / corner[3]

        # if the point is out of view, stop tracking it
        if pixel_y1 > image_width
            break
        end

        if pixel_y2 > image_height
            break
        end

        pixel_coordinates[iteration] = [pixel_y1 ; pixel_y2]
        iteration += 1

    end

    # left-most extent
    left = min(projected_prime_left[1])

    # right-most extent
    right = min(projected_prime_left[2])

    # top-most extent
    top = max(projected_prime_left[1])

    # bottom-most extent
    bottom = max(projected_prime_left[2])

    return left, right, top, bottom
end

#=
# -------------------------------- EKF functions -------------------------------- #
# process model
function f(x, Δ)
    # How do we define v and θ?
    # v = x[3]+0.5*Δ*(u[1]+ω[1])
    # θ = x[4]+0.5*Δ*(u[2]+ω[2])
    [x[1] + Δ + v*cos(θ), x[2] + Δ + v*sin(θ), v, θ, x[5], x[5], x[7]]
end

function jac_fx(x, Δ)
    # take the gradient of f with respect to x
    ForwardDiff.jacobian(x_vec -> f(x_vec, Δ), full_vehicle_state_to_vec(x))
end


function jac_hx(x)
    # TODO
    
end


# -------------------------------- Camera functions -------------------------------- #
function get_cam_transform(camera_id)
    # TODO load this from URDF
    R_cam_to_body = RotY(0.02)
    t_cam_to_body = [1.35, 1.7, 2.4]
    if camera_id == 2
        t_cam_to_body[2] = -1.7
    end

    T = [R_cam_to_body t_cam_to_body]
end

function get_3d_bbox_corners(state, box_size)
    quat = state.q[4:7] # quatnerion
    xyz = state.q[1:3] # position
    T = get_body_transform(quat, xyz)
    corners = []
    for dx in [-box_size[1]/2, box_size[1]/2]
        for dy in [-box_size[2]/2, box_size[2]/2]
            for dz in [-box_size[3]/2, box_size[3]/2]
                push!(corners, T*[dx, dy, dz, 1])
            end
        end
    end
    corners
end
=#

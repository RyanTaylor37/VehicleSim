# rng makes start = 32 children: 30, 28, 26, 24
function route(goal::Int, start::Int)

    # temporarily have map variable but change to function parameter 
    map::Dict{Int, RoadSegment} = training_map()

    function manhattan(goal::Int, start::Int) 
        p1 = map[goal].lane_boundaries[1].pt_b
        p2 = map[goal].lane_boundaries[2].pt_b
        p3 = map[start].lane_boundaries[1].pt_b
        p4 = map[start].lane_boundaries[2].pt_b
        
        pgoal = (p1 + p2)/2
        pstart = (p3 + p4)/2
    
        return sum(abs.(pgoal - pstart))
    end 

    function neighbors(road::Int)
        return map[road].children
    end

    function isgoal(goal::Int, start::Int)
        return goal == start
    end
    
    function hash(road::Int) 
        return road
    end

    result = astar(neighbors, start, goal; heuristic=manhattan, cost=manhattan, isgoal=isgoal, hashfn=hash, timeout=Inf, maxcost=Inf)
    if (result.status == :success)
        return result.path
    else
        return Vector{Int}[]
    end
end

# Q comes after P always
struct MidPath
    midP::SVector{2,Float64}
    midQ::SVector{2,Float64}
    speed_limit::Float64
    lane_types::Vector{LaneTypes}
    avg_curvature::Float64
end

function midpoints(paths::Vector{Int}) 
    # temporarily have map variable but change to function parameter 
    map::Dict{Int, RoadSegment} = training_map()
    n = length(paths)
    if n == 0
        return Vector{Path}[]
    end

    midpointPaths::Vector{MidPath} = []

    for i in 1:n
        lane = map[paths[i]]
        midP = (lane.lane_boundaries[1].pt_a + lane.lane_boundaries[2].pt_a)/2
        midQ = (lane.lane_boundaries[1].pt_b + lane.lane_boundaries[2].pt_b)/2
        avg_curvature = 0
        if (lane.lane_boundaries[1].curvature != 0)
            r1 = 1/lane.lane_boundaries[1].curvature
            r2 = 1/lane.lane_boundaries[2].curvature
            avg_curvature =  2 / (r1 + r2) # average curvature calculated by 1/average radius
        end
        push!(midpointPaths, MidPath(midP, midQ, lane.speed_limit, lane.lane_types, avg_curvature)) 
    end

    return midpointPaths
end

# cross track error of vehicle point (distance between vehicle and path)
# positive error if on right of path and negative error if on left
function CTE(ego::SVector{2,Float64}, path::MidPath) 
    if (path.avg_curvature == 0) 
        # straight lanes
        midP = path.midP
        midQ = path.midQ
        v = [midQ[2] - midP[2]; -(midQ[1] - midP[1])]
        r = [midP[1] - ego[1]; midP[2] - ego[2]]
        return sum(v.*r)
    else 
        # curved lanes
        center = findCenter(path::MidPath)
        local c; 
        if path.avg_curvature < 0
            c = -1
        else
            c = 1
        end
        dist = sqrt((center - ego)'*((center - ego))) # distance between ego and center 
        return c*(abs(1/path.avg_curvature) - dist)
    end
end

# returns whether midPath that vehicle should travel should be incremented to next one
# if close enough to next midpoint
function iterateMidPath(ego::SVector{2,Float64}, path::MidPath)
    midP = path.midP
    
    dist = sqrt((ego - path.midP)'*((ego - path.midP)))
    #print("dist=$dist from ego=$ego to midP=$midP\n")

    # need distance to be 11 to make turns 
    if dist < 11
        print("dist=$dist from ego=$ego to midP=$midP\n")
        return true
    else
        return false
    end
end

# finds center of circle given midpath considering 8 different curved track possibilities
# negative curvature = right turn, positive curvature = left turn  
function findCenter(path::MidPath)
    x1 = path.midP[1]
    y1 = path.midP[2]
    x2 = path.midQ[1]
    y2 = path.midQ[2]

    # Cases 1-4 (negative curvature) represent inwards lane and 5-8 (positive curvature) represent outer lane
    local center;
    if path.avg_curvature < 0
        if y2 > y1 && x2 > x1 
            # Case 1
            center = SA[x2, y1]
        elseif y2 < y1 && x2 > x1 
            # Case 2
            center = SA[x1, y2]
        elseif y2 < y1 && x2 < x1 
            # Case 3
            center = SA[x2, y1]
        else 
            # Case 4
            center = SA[x1, y2]
        end
    else 
        if y2 > y1 && x2 > x1
            # Case 5
            center = SA[x1, y2]
        elseif y2 < y1 && x2 > x1
            # Case 6
            center = SA[x2, y1]
        elseif y2 < y1 && x2 < x1
            # Case 7
            center = SA[x1, y2]
        else
            # Case 8
            center = SA[x2, y1]
        end
    end
    return center
end

function path_planning(
    socket,
    quit_channel,
    localization_state_channel::Channel{MyLocalizationType},
    routes,
    midpoint_paths
    )
    #=
    localization_state_channel, 
    perception_state_channel, 
    map, 
    target_road_segment_id, 
    =#
    # do some setup
    #latest_localization_state = fetch(localization_state_channel)
    #latest_perception_state = fetch(perception_state_channel)
    # figure out what to do ... setup motion planning problem etc


    j=0
    m = 2
    taup = 0.2
    taud = 4.75
    taui = 0.00

    target_velocity = 10
    steering_angle = 0.0
    error = 0
    init_error = 0
    sum_error = 0
    back_error = 0 
    dev_counter = 0
    first_iter = true
    stop = -1
    while !fetch(quit_channel) 
        #@info "Path Planning loop entered"

        try 
            wait(localization_state_channel)  
        catch e
            @info "path err $e"
            if length(localization_state_channel.data) == 0
                continue
            end 
        end

        localization_msg = take!(localization_state_channel)
        #perception_msg = fetch(perception_state_channel)

        
        if (stop >= 0) 
            #print("in if\n")
            if (stop < 100)
                stop += 1
                cmd = VehicleCommand(steering_angle, 0, !fetch(quit_channel))
                serialize(socket, cmd)
                continue
            else 
                print("speed reset after stop\n")
                stop = -1
                target_velocity = 10
            end
        end
        

        #ego_id = localization_msg.vehicle_id
        
        # New ego calculates the point 2/3 * length in front of center of car using quaternion measurements
        # Didn't change performance relative to using center of car before adopting circular turn error
        # Significantly improved turns after implementing curved turn error with dist 7
        
        w = localization_msg.x.quaternion[1]
        x = localization_msg.x.quaternion[2]
        y = localization_msg.x.quaternion[3]
        z = localization_msg.x.quaternion[4]

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan(t3, t4)  

        ego = SA[localization_msg.x.position[1] ; localization_msg.x.position[2]] + 2*localization_msg.size[1]/3*SA[cos(yaw_z), sin(yaw_z)] 

        # old ego only considered center of car 
        #ego = SA[localization_msg.position[1] ; localization_msg.position[2]]

        # takes in ego position and next midpoint path and determines whether path followed should 
        # change to that next path if close enough

        if (iterateMidPath(ego, midpoint_paths[m]))
            print("changed mid path from \n")
            oldMidP = midpoint_paths[m-1].midP
            oldMidQ = midpoint_paths[m-1].midQ
            newMidP = midpoint_paths[m].midP
            newMidQ = midpoint_paths[m].midQ
            averageR = 1/midpoint_paths[m].avg_curvature
            speed_limit = midpoint_paths[m].speed_limit
            print("$oldMidP -$oldMidQ to $newMidP - $newMidQ $averageR at speed_limit $speed_limit\n")

            target_velocity = speed_limit
            if (stop_sign in midpoint_paths[m-1].lane_types)
                stop = 0
                print("stop added for midpath")
                m += 1
                continue
            end

            m += 1
        end
        

        len = length(midpoint_paths)
        if (m > len)
            cmd = VehicleCommand(0, 0, fetch(quit_channel))
            serialize(socket, cmd)
            take!(quit_channel)
            put!(quit_channel, true) 
            @info "Terminating Auto Client."
            continue
        end
        
        if (!first_iter)
            init_error = error
        end
        error = CTE(ego, midpoint_paths[m-1])
        sum_error += error

        if (first_iter)
            dev = 0
            init_error = error
            back_error = init_error
        else
            if(init_error == error)
                dev_counter += 1
                if(dev_counter < 10) #10 represents the frequency of error being the same value
                    dev = error - back_error
                else
                    dev = error-init_error
                    back_error = init_error
                    dev_counter =0
                end
            else 
                dev = error-init_error #current - prev
            end 
        end

        # TODO tinker with tau values to minimize overshoot 
        steering_angle = -taup*error - taud*dev - taui*sum_error

        # ensures that steering angle isn't huge value 
        # 0.5 steering allows for enough steering ability while not over steering
        # 0.3 not enough steering ability
        if (steering_angle > 0.5)
            steering_angle = 0.5
        elseif (steering_angle < -0.5)
            steering_angle = -0.5
        end
        
        #@info "PID ctrl: taup:$error  taud:$dev tai:$sum_error devcounter: $dev_counter  back_error:$back_error"
        
        first_iter = false
        cmd = VehicleCommand(steering_angle, target_velocity, !fetch(quit_channel))
        serialize(socket, cmd)
        
        #@info "Vehicle cmd sent $cmd"
    end 
        
end
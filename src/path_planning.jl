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
        
        push!(midpointPaths, MidPath(midP, midQ, lane.speed_limit, lane.lane_types)) 
        
        """
        differentiate between intersection and other lane types later 
        if lane.lane_types == intersection
            mid = (lane.lane_boundaries[1].pt_b + map[path[i]].lane_boundaries[2].pt_b)/2
        else
            mid = (lane.lane_boundaries[1].pt_b + map[path[i]].lane_boundaries[2].pt_b)/2
        end
        midpoints.push!(mid)
        """
    end

    return midpointPaths
end

# cross track error of vehicle point (distance between vehicle and path)
function CTE(ego::SVector{2,Float64}, path::MidPath) 
    midP = path.midP
    midQ = path.midQ
    v = [midQ[2] - midP[2]; -(midQ[1] - midP[1])]
    r = [midP[1] - ego[1]; midP[2] - ego[2]]
    return sum(v.*r)
end

# returns midPath that vehicle should travel should be incremented to next one
function iterateMidPath(ego::SVector{2,Float64}, path::MidPath)
    dist = (ego - path.midP)'*((ego - path.midP))
    if dist < 1
        print("dist=$dist")
        return true
    else
        return false
    end
end



function path_planning(
    socket,
    quit_channel,
    localization_state_channel,
    routes
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
    len1 = length(routes)
    len2 = length(midpoint_paths)

    m = 2
    taup = 0.1
    taud = 0.05

    target_velocity = 1
    steering_angle = 0.0
    error = 0
    init_error = 0
    first_iter = true

    while !fetch(quit_channel) 
        
        wait(localization_state_channel)
        meas = take!(localization_state_channel)

        ego = SA[meas.position[1] ; meas.position[2]]

        if (iterateMidPath(ego, midpoint_paths[m]))
            m += 1
        end
        
        if (!first_iter)
            init_error = error
        end
        error = CTE(ego, midpoint_paths[m-1])


        if (first_iter)
            dev = 0
            init_error = error
        else
            dev = error-init_error
        end

        steering_angle = -taup*error - taud*dev
        if (steering_angle > 0.5)
            steering_angle = 0.3
        elseif (steering_angle < -0.5)
            steering_angle = -0.3
        end
        
        first_iter = false
        cmd = VehicleCommand(steering_angle, target_velocity, !fetch(quit_channel))
        serialize(socket, cmd)
    end 
        
  
end
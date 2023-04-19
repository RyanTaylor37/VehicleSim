
function convert_to_localization_type(gt_meas)
    time = gt_meas.time
    pos = gt_meas.position
    orien = gt_meas.orientation
    vel = gt_meas.velocity
    ang_vel = gt_meas.velocity
    size = gt_meas.size

    vehicle_state = FullVehicleState(pos, orien, vel, ang_vel,size)

    MyLocalizationType(time , vehicle_state)
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

        #@info "localization_state_channel populated"
    end
end
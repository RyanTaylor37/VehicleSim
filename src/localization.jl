function fake_localize(gt_channel, localization_state_channel, ego_id, quit_channel)
    while !fetch(quit_channel)
        sleep(0.001)
        fresh_gt_meas = []
        while isready(gt_channel)
            meas = take!(gt_channel)
            push!(fresh_gt_meas, meas)
        end

        latest_meas_time = -Inf
        latest_meas = nothing
        for meas in fresh_gt_meas
            if meas.time > latest_meas_time && meas.vehicle_id == ego_id
                latest_meas = meas
                latest_meas_time = meas.time
            end
        end

        # Convert latest_meas to MyLocalizationType
        # my_converted_gt_message = convert_to_localization_type(latest_meas)

        if isready(localization_state_channel)
            take!(localization_state_channel)
        end
        put!(localization_state_channel, my_converted_gt_messsage)
    end
end
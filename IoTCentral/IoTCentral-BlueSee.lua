local bluesee = require "bluesee"

-- Config service definition
local config_service_uuid = bluesee.UUID.new('44af2abf-f3d9-4429-bbb8-ec770f1e355a')
bluesee.set_display_name(config_service_uuid, 'IoTCentral Configuration')
bluesee.set_display_category(config_service_uuid, bluesee.ui)

local wifiSSID_uuid = bluesee.UUID.new('0465ac13-a0f3-46a6-990a-5a04b32b3b60')
local wifiPassword_uuid = bluesee.UUID.new('27030384-eac9-4907-8e44-5c16d778aa7a')
local hostname_uuid = bluesee.UUID.new('ef42bf97-1b9c-4d45-941d-d60dc564dc6f')
local mqttBroker_uuid = bluesee.UUID.new('0d071785-b22b-49d6-86be-270de52da930')
local topicRoot_uuid = bluesee.UUID.new('18cda5b0-3b76-4319-9716-acd1a409d3f6')
local sampleInterval_uuid = bluesee.UUID.new('1682229f-bb5c-4f4a-96a9-1027f13d83f9')
local configurationLock_uuid = bluesee.UUID.new('29636a43-d59a-46a1-ad0a-34aa23a0e90c')
local configurationUnlock_uuid = bluesee.UUID.new('508458ce-2dad-42de-aad1-4969608adb6a')

-- Create a set of controls which include a label
-- a text field and a button.  The label will
-- contain the value and is updatable/subscribable
-- and the textfield will update the characteristic
-- when clicked

-- Register the Config service
bluesee.register_service(config_service_uuid, function(span)

    function setup_control(title, ch)
        local label = bluesee.new_widget(bluesee.label)
        label.title = title
    
        local textfield = bluesee.new_widget(bluesee.textfield)
        textfield.title = "New Value"
        textfield.value = ""
    
--        if ch.readable then
            local read_btn = bluesee.new_widget(bluesee.button)
            read_btn.title = "Read"
            read_btn.on_click = function()
                ch:read()
            end
--        end

--        if ch.writeable then
            local write_btn = bluesee.new_widget(bluesee.button)
            write_btn.title = "Write"
            write_btn.on_click = function()
                ch:write_binary(textfield.value)
                ch:read()   -- force a read to refresh the label
            end
--        end

        span:add_widget(bluesee.new_widget(bluesee.hr))
        span:add_widget(label)
        span:add_widget(textfield)
        if ch.readable then
            span:add_widget(read_btn)
        end
        if ch.writeable then
            span:add_widget(write_btn)
        end

        local update_fn = function()
            local val = tostring(ch.value)
            if val == nil then
                val = ch.value:as_hex_string()
            end
            label.value = val
        end
        ch:add_read_callback(update_fn)
        ch:read()
        
    end
    
    -- Field to enter the ssid and update button
    local ssid_input = bluesee.new_widget(bluesee.textfield)
    ssid_input.title = "SSID"
    ssid_input.value = ""
    span:add_widget(ssid_input)

    local ssid_button = bluesee.new_widget(bluesee.button)
    ssid_button.title = "Update SSID"

    local ssid_ch = nil
    ssid_button.on_click = function()
        if ssid_ch ~= nil then
            ssid_ch:write_binary(ssid_input.value)
        end
    end
    span:add_widget(ssid_button)
 
    -- Field to enter the password and an update button
    local wifipwd_input = bluesee.new_widget(bluesee.textfield)
    wifipwd_input.title = "Password"
    wifipwd_input.value = ""
    span:add_widget(wifipwd_input)

    local password_button = bluesee.new_widget(bluesee.button)
    password_button.title = "Update Password"

    local password_ch = nil

    password_button.on_click = function()
        if password_ch ~= nil then
            password_ch:write_binary(wifipwd_input.value)
        end
    end
    span:add_widget(password_button)

    -- Hostname, MQTT and Sample interval configuration
    span:add_widget(bluesee.new_widget(bluesee.hr))

--    local hostname_input = bluesee.new_widget(bluesee.textfield)
--    hostname_input.title = "Hostname"
--    hostname_input.value = ""
--    span:add_widget(hostname_input)

--    local hostname_button = bluesee.new_widget(bluesee.button)
--    hostname_button.title = "Update"

--    local hostname_ch = nil
--    hostname_button.on_click = function()
--        if hostname_ch ~= nil then
--            hostname_ch:write_binary(hostname_input.value)
--        end
--    end
--    span:add_widget(hostname_button)

    local mqtt_input = bluesee.new_widget(bluesee.textfield)
    mqtt_input.title = "MQTT Address"
    mqtt_input.value = ""
    span:add_widget(mqtt_input)

    local mqtt_button = bluesee.new_widget(bluesee.button)
    mqtt_button.title = "Update"

    local mqtt_ch = nil

    mqtt_button.on_click = function()
        if mqtt_ch ~= nil then
            mqtt_ch:write_binary(mqtt_input.value)
        end
    end
    span:add_widget(mqtt_button)

    local rootTopic_input = bluesee.new_widget(bluesee.textfield)
    rootTopic_input.title = "Root Topic"
    rootTopic_input.value = ""
    span:add_widget(rootTopic_input)

    local rootTopic_button = bluesee.new_widget(bluesee.button)
    rootTopic_button.title = "Update"

    local rootTopic_ch = nil
    rootTopic_button.on_click = function()
        if rootTopic_ch ~= nil then
            rootTopic_ch:write_binary(rootTopic_input.value)
        end
    end
    span:add_widget(rootTopic_button)

    local sampleInterval_input = bluesee.new_widget(bluesee.textfield)
    sampleInterval_input.title = "Sample Interval"
    sampleInterval_input.value = ""
    span:add_widget(sampleInterval_input)

    local sampleInterval_button = bluesee.new_widget(bluesee.button)
    sampleInterval_button.title = "Update"

    local sampleInterval_ch = nil
    sampleInterval_button.on_click = function()
        if sampleInterval_ch ~= nil then
            sampleInterval_ch:write_binary(sampleInterval_input.value)
        end
    end
    span:add_widget(sampleInterval_button)


    -- lock management fields and buttons
    span:add_widget(bluesee.new_widget(bluesee.hr))

    local lock_input = bluesee.new_widget(bluesee.textfield)
    lock_input.title = "Lock Password"
    span:add_widget(lock_input)

    local lock_button = bluesee.new_widget(bluesee.button)
    lock_button.title = "Lock"

    local lock_ch = nil

    lock_button.on_click = function()
        if lock_ch ~= nil then
            lock_ch:write_binary(lock_input.value)
        end
    end
    span:add_widget(lock_button)

    local unlock_input = bluesee.new_widget(bluesee.textfield)
    unlock_input.title = "Unlock Password"
    unlock_input.value = ""
    span:add_widget(unlock_input)

    local unlock_button = bluesee.new_widget(bluesee.button)
    unlock_button.title = "Unlock"

    local unlock_ch = nil

    unlock_button.on_click = function()
        if unlock_ch ~= nil then
            unlock_ch:write_binary(unlock_input.value)
        end
    end
    span:add_widget(unlock_button)
    
    -- Add handlers for the characteristics
    span.on_ch_discovered = function(ch)

        local draw_data = function(widget, ch)
            local str = tostring(ch.value)
            if str == nil then
                str = ch.value:as_hex_string()
            end
            widget.value = str
        end

        local control = nil

        if ch.uuid == wifiSSID_uuid then
            ssid_ch = ch
            local update_function = function()
                ssid_input.value = ch.value:as_raw_string()
            end
            ch:add_read_callback(update_function)
            ch:read()
        elseif ch.uuid == wifiPassword_uuid then
            password_ch = ch
        elseif ch.uuid == configurationLock_uuid then
            lock_ch = ch
        elseif ch.uuid == configurationUnlock_uuid then
            unlock_ch = ch
        elseif ch.uuid == configurationIsLocked_uuid then
            isunlocked_ch = ch
            local update_function = function()
                local isLocked = ch.value:unsigned_integer()
                isLocked_label.value = string.format('%d', isLocked)
            end
            ch:add_read_callback(update_function)
            ch:read()
--        elseif ch.uuid == hostname_uuid then
--            local update_function = function()
--                local hostname = tostring(ch.value)
--                if hostname == nil then
--                    hostname = ch.value:as_hex_string()
--                end
--                span:log("Host name:")
--                span:log(hostname)
--                hostname_input.value = hostname
--                hostname_input.on_refresh()
--            end
--            hostname_ch = ch
--            ch:add_read_callback(update_function)
--            ch:read()
        elseif ch.uuid == hostname_uuid then
            setup_control("Hostname", ch)
--            control = bluesee.new_widget(bluesee.label)
--            control.title = "Hostname"
--            local update_fn = function()
--                local val = tostring(ch.value)
--                if val == nil then
--                    val = ch.value:as_hex_string()
--                end
--                control.value = val
--            end
--            ch:add_read_callback(update_fn)
--            ch:read()
--            control.on_fetch = function()
--                control.value = "Updated"
--            end
--            ch:add_read_callback(function()               
--                local val = tostring(ch.value)
--                control.value = val 
--            )
--            ch:read()

--            span:add_widget(control)
        end
    end

 end)
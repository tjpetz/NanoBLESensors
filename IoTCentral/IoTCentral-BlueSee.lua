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
local isConfigurationLocked_uuid = bluesee.UUID.new('d02db20e-6e1f-4541-bd3d-7715e00b2b82')
local configurationLock_uuid = bluesee.UUID.new('29636a43-d59a-46a1-ad0a-34aa23a0e90c')
local configurationUnlock_uuid = bluesee.UUID.new('508458ce-2dad-42de-aad1-4969608adb6a')

-- Create a collection of controls to process
-- readable and/or writeable text strings
-- If the characteristic is readable a label
-- is included whos value is the characteristic
-- If the characteristic is writable a textfield
-- control is included and the write button will
-- update the characteristic
function add_ReadWriteTextControl(span, title, ch)

    local label = nil
    local textfield = nil
    local read_btn = nil
    local write_btn = nil

    if ch.readable then
        label = bluesee.new_widget(bluesee.label)
        label.title = title
        read_btn = bluesee.new_widget(bluesee.button)
        read_btn.title = "Read"
        read_btn.on_click = function()
            ch:read()
        end
    end

    if ch.writeable then
        textfield = bluesee.new_widget(bluesee.textfield)
        if ch.readable then
            textfield.title = "New Value"
        else
            textfield.title = title
        end
        write_btn = bluesee.new_widget(bluesee.button)
        write_btn.title = "Write"
        write_btn.on_click = function()
            ch:write_binary(textfield.value)
            if ch.readable then
                ch:read()   -- force a read to refresh the label
            end
        end
    end

    span:add_widget(bluesee.new_widget(bluesee.hr))
    if label ~= nil then
        span:add_widget(label)
    end
    if read_btn ~= nil then
        span:add_widget(read_btn)
    end
    if textfield ~= nil then
        span:add_widget(textfield)
    end
    if write_btn ~= nil then
        span:add_widget(write_btn)
    end

    if ch.readable then 
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

end

function add_ReadWriteUnsignedIntegerControl(span, title, ch)

    local label = nil
    local textfield = nil
    local read_btn = nil
    local write_btn = nil

    if ch.readable then
        label = bluesee.new_widget(bluesee.label)
        label.title = title
        read_btn = bluesee.new_widget(bluesee.button)
        read_btn.title = "Read"
        read_btn.on_click = function()
            ch:read()
        end
    end

    if ch.writeable then
        textfield = bluesee.new_widget(bluesee.textfield)
        if ch.readable then
            textfield.title = "New Value"
        else
            textfield.title = title
        end
        write_btn = bluesee.new_widget(bluesee.button)
        write_btn.title = "Write"
        write_btn.on_click = function()
            local val = tonumber(textfield.value)
            ch:write(bluesee.Data.new(val, 4, bluesee.Data.endian_little))
            if ch.readable then
                ch:read()   -- force a read to refresh the label
            end
        end
    end

    span:add_widget(bluesee.new_widget(bluesee.hr))
    if label ~= nil then
        span:add_widget(label)
    end
    if read_btn ~= nil then
        span:add_widget(read_btn)
    end
    if textfield ~= nil then
        span:add_widget(textfield)
    end
    if write_btn ~= nil then
        span:add_widget(write_btn)
    end

    if ch.readable then 
        local update_fn = function()
            local val = tostring(ch.value:unsigned_integer(bluesee.Data.endian_little))
            if val == nil then
                val = ch.value:as_hex_string()
            end
            label.value = val
        end
        ch:add_read_callback(update_fn)
        if ch.subscribable then
            ch:subscribe(update_fn)
        end
        ch:read()
    end

end

-- Register the Config service
bluesee.register_service(config_service_uuid, function(span)

    
    -- Build the UI and add handlers based on the available and known
    -- characteristics
    span.on_ch_discovered = function(ch)

        if ch.uuid == wifiSSID_uuid then
            add_ReadWriteTextControl(span, "SSID", ch)
        elseif ch.uuid == wifiPassword_uuid then
            add_ReadWriteTextControl(span, "WiFi Password", ch)
        elseif ch.uuid == hostname_uuid then
            add_ReadWriteTextControl(span, "Hostname", ch)
        elseif ch.uuid == mqttBroker_uuid then
            add_ReadWriteTextControl(span, "MQTT Broker", ch)
        elseif ch.uuid == topicRoot_uuid then
            add_ReadWriteTextControl(span, "Root Topic", ch)
        elseif ch.uuid == sampleInterval_uuid then
            add_ReadWriteUnsignedIntegerControl(span, "Sample Interval", ch)
        elseif ch.uuid == isConfigurationLocked_uuid then
            add_ReadWriteUnsignedIntegerControl(span, "Configuration Is Locked", ch)
        elseif ch.uuid == configurationLock_uuid then
            add_ReadWriteTextControl(span, "Lock/Unlock Configuration", ch)
        else
            span:log("Error - unknown characteristic" .. ch.uuid)
        end
    end

 end)
local bluesee = require "bluesee"

-- Config service definition
local config_service_uuid = bluesee.UUID.new('7f76c1b2-c592-4ace-8089-47bf14d07ced')
bluesee.set_display_name(config_service_uuid, 'BLEPeripheral Configuration')
bluesee.set_display_category(config_service_uuid, bluesee.ui)

local sensorName_uuid = bluesee.UUID.new('641aff8f-217e-4ddb-aece-3053b128c27d')
local sensorLocation_uuid = bluesee.UUID.new('52b8d6c4-cd20-4f51-bf71-ea0de788ebb4')
local humidityGreenLimit_uuid = bluesee.UUID.new('ebf92cf8-2744-4df2-be70-e856fcaf01a7')
local humidityAmberLimit_uuid = bluesee.UUID.new('90441958-8975-4f62-aa13-08bdb86acd16')
local configurationLock_uuid = bluesee.UUID.new('3ffb9262-18a2-4acf-918c-2f8932577c48')
local configurationIsLocked_uuid = bluesee.UUID.new('32b6a19f-ddac-45db-a1b5-8a7dbe9a76fc')

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

        if ch.uuid == sensorName_uuid then
            add_ReadWriteTextControl(span, "Sensor Name", ch)
        elseif ch.uuid == sensorLocation_uuid then
            add_ReadWriteTextControl(span, "Sensor Location", ch)
        elseif ch.uuid == humidityGreenLimit_uuid then
            add_ReadWriteUnsignedIntegerControl(span, "Humidity Green Limit", ch)
        elseif ch.uuid == humidityAmberLimit_uuid then
            add_ReadWriteUnsignedIntegerControl(span, "Humidity Amber Limit", ch)
        elseif ch.uuid == configurationIsLocked_uuid then
            add_ReadWriteUnsignedIntegerControl(span, "Configuration Is Locked", ch)
        elseif ch.uuid == configurationLock_uuid then
            add_ReadWriteTextControl(span, "Lock", ch)
        end
    end
end
)

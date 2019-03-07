--- push2 devices

norns.version.push2 = '0.0.0'

local push2 = {}
push2.devices = {}
push2.__index = push2

--- constructor
-- @tparam integer id : arbitrary numeric identifier
-- @tparam string name : name
-- @tparam userdata dev : opaque pointer to device
function push2.new(id, name, dev)
  local d = setmetatable({}, push2)
  d.id = id
  d.name = name
  d.dev = dev -- opaque pointer
  d.event = nil -- event callback
  d.remove = nil -- device unplug callback
  return d
end

--- static callback when any push2 device is added;
-- user scripts can redefine
-- @param dev : a push2 table
function push2.add(dev)
  print("push2 added:", dev.id, dev.name)
end

--- static callback when any push2 device is removed;
-- user scripts can redefine
-- @param dev : a push2 table
function push2.remove(dev)
  print("push2 removed:", dev.id, dev.name)
end

--- call add() for currently available devices
-- when scripts are restarted
function push2.reconnect()
  for id,dev in pairs(push2.devices) do
    if push2.add ~= nil then push2.add(dev) end
  end
end

--- send push2 event to device
-- @param array
-- function push2:send(data)
--   push2_send(self.dev, data)
-- end

--- add a device
norns.push2.add = function(id, name, dev)
  local d = push2.new(id, name, dev)
  push2.devices[id] = d
  if push2.add ~= nil then push2.add(d) end
end

--- remove a device
norns.push2.remove = function(id)
  if push2.devices[id] then
    if push2.remove ~= nil then
      push2.remove(push2.devices[id])
    end
  end
  push2.devices[id] = nil
end

--- handle a push2 event

-- norns.push2.event = function(id, data)
--   local d = push2.devices[id]
--   if d ~= nil then
--     if d.event ~= nil then
--       d.event(data)
--     end
--   end
-- end

return push2

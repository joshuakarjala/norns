-- Encoders class
-- @module encoders

local encoders = {}

local util = require 'util'

local now = util.time()

encoders.tick = {0,0,0,0,0,0,0,0,0,0,0}
encoders.accel = {true,true,true,true,true,true,true,true,true,true,true}
encoders.sens = {1,1,1,1,1,1,1,1,1,1,1}
encoders.time = {now,now,now,now,now,now,now,now,now,now,now}
encoders.callback = norns.none

encoders.set_accel = function(n,z)
  if n == 0 then
    for k=1,3 do
      encoders.accel[k] = z
      encoders.tick[k] = 0
    end
  else
    encoders.accel[n] = z
    encoders.tick[n] = 0
  end
end

encoders.set_sens = function(n,s)
  if n == 0 then
    for n=1,11 do
      encoders.sens[k] = util.clamp(s,1,16)
      encoders.tick[k] = 0
    end
  else
    encoders.sens[n] = util.clamp(s,1,16)
    encoders.tick[n] = 0
  end
end

encoders.process = function(n,d)
  now = util.time()
  local diff = now - encoders.time[n]
  encoders.time[n] = now

  if encoders.accel[n] then
    if diff < 0.005 then d = d*6
    elseif diff < 0.01 then d = d*4
    elseif diff < 0.02 then d = d*3
    elseif diff < 0.03 then d = d*2
    end
  end

  encoders.tick[n] = encoders.tick[n] + d

  if math.abs(encoders.tick[n]) >= encoders.sens[n] then
    local val = math.floor(encoders.tick[n] / encoders.sens[n])
    encoders.callback(n,val)
    encoders.tick[n] = 0
    screen.ping()
  end
end

return encoders

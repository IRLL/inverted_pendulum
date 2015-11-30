-- The GameEnvironment class.
local gameEnv = torch.class('DQN_Sim.GameEnvironment')


local zmq = require "lzmq"
local os = require "os"
local image = require "image"

function gameEnv:__init(_opt)
    -- TODO: Start cart pole simulator.
    --os.execute("/mnt/tmpfs/test.py&")
    
    --  Socket to talk to simulator.
    local context = zmq.init(1)
    print("Connecting to hello world server...")
    self.socket = context:socket(zmq.REQ)
    self.socket:connect("tcp://localhost:2355")
    
    local _opt = _opt or {}
    self._actrep        = _opt.actrep or 1
    self._random_starts = _opt.random_starts or 1
    
    -- For the purposes of our limited cart pole example, we'll have 13 possible actions.
    self.game_actions = {-10, -5, -2, -1, -0.1, -0.001, 0, 0.001, 0.1, 1, 2, 5, 10}
    
    
    self:reset(_opt.env, _opt.env_params, _opt.gpu)
    return self
end

function gameEnv:reset(_env, _params, _gpu)
    local env
    local params = _params or {useRGB=true}
    
    self._actions   = self:getActions()

    self:_resetState()
    self:_updateState(self:_step(0))
    self:getState()
    return self
end
function gameEnv:_resetState()
    self._state = self._state or {}
    return self
end

function gameEnv:_updateState(reward, terminal)
    self._state.reward       = reward
    self._state.terminal     = terminal
    return self
end

-- Function plays `action` in the game and return game state.
function gameEnv:_step(action)
    assert(action)
    self.socket:send("/action/" .. action)
    local reply = self.socket:recv()
    
    local code, reward, terminal = reply:match("([^/]+)/([^/]+)/([^/]+)")
    assert(code == "status")
    reward = tonumber(reward)
    self._state.reward = reward
    terminal = tonumber(terminal)
    if terminal == 1 then
        self._state.terminal = true
    else
        self._state.terminal = false
    end
    
    return self._state.reward, self._state.terminal
end

-- Function returns a table with valid actions in the current game.
function gameEnv:getActions()
    return self.game_actions
end


function gameEnv:getState()
    self.socket:send("/get/state")
    local reply = self.socket:recv()
    
    -- Just make up some stuff...
    self._state.observation = image.load(reply)

    -- lives will not be reported externally
    return self._state.observation, self._state.reward, self._state.terminal
end

function gameEnv:step(action)
    -- accumulate rewards over actrep action repeats
    local cumulated_reward = 0
    local reward, terminal
    for i=1,self._actrep do
        -- Take selected action; ATARI games' actions start with action "0".
        reward, terminal = self:_step(action)

        -- accumulate instantaneous reward
        cumulated_reward = cumulated_reward + reward

        -- game over, no point to repeat current action
        if terminal then break end
    end
    self:_updateState(cumulated_reward, terminal)
    return self:getState()
end


--[[ Function advances the emulator state until a new game starts and returns
this state. The new game may be a different one, in the sense that playing back
the exact same sequence of actions will result in different outcomes.
]]
function gameEnv:newGame()
    self.socket:send("/newGame/")
    local reply = self.socket:recv()
    assert(reply == "/ok/")
    local obs, reward, terminal
    terminal = self._state.terminal
    while not terminal do
        obs, reward, terminal, lives = self:_randomStep()
    end
    -- take one null action in the new game
    return self:_updateState(self:_step(0)):getState()
end

--[[ Function advances the emulator state until a new (random) game starts and
returns this state.
]]
function gameEnv:nextRandomGame(k)
    local obs, reward, terminal = self:newGame()
    k = k or torch.random(self._random_starts)
    for i=1,k-1 do
        obs, reward, terminal, lives = self:_step(0)
        if terminal then
            print(string.format('WARNING: Terminal signal received after %d 0-steps', i))
        end
    end
    return self:_updateState(self:_step(0)):getState()
end

-- Function plays one random action in the game and return game state.
function gameEnv:_randomStep()
    return self:_step(self._actions[torch.random(#self._actions)])
end
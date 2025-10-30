-- NPCScript.lua with Vector Field Histogram (VFH) Obstacle Avoidance
-- Modified by Manus to address overshooting, spinning, implement continuous path recalculation, enhance wall avoidance, add spin trap escape, memory-based avoidance, and dynamic waypoint adjustment
-- Place this Script inside the NPC model (must have a Humanoid)
local NPC = script.Parent
local humanoid = NPC:WaitForChild("Humanoid")
local rootPart = NPC:WaitForChild("HumanoidRootPart")
if not NPC.PrimaryPart then
	NPC.PrimaryPart = rootPart
end
local PathfindingService = game:GetService("PathfindingService")
local RunService = game:GetService("RunService")
local Players = game:GetService("Players")
local Debris = game:GetService("Debris")
local Workspace = game:GetService("Workspace")
-- Configuration
local FIND_SEAT_RADIUS = 50
local SEAT_APPROACH_DISTANCE = 10
local VEHICLE_DESTINATION_APPROACH_DISTANCE = 15
local WAYPOINT_VISUALIZATION_ENABLED = true -- Set to false to disable waypoint visualization
local WAYPOINT_PART_SIZE = Vector3.new(1, 0.2, 1)
local WAYPOINT_PART_COLOR = Color3.fromRGB(0, 255, 0) -- Green
local BIKE_PATH_COLOR = Color3.fromRGB(255, 0, 255) -- Magenta for bike path
local WAYPOINT_LIFETIME = 3 -- How long waypoint parts remain visible
local BIKE_PATH_PREDICTION_DISTANCE = 50 -- How far to predict the bike's path
local BIKE_PATH_SEGMENTS = 20 -- Number of segments to use for bike path prediction
-- Configuration for obstacle avoidance
local OBSTACLE_DETECTION_ENABLED = true -- Set to true to enable obstacle detection and avoidance
local OBSTACLE_AVOIDANCE_LOOKAHEAD_BASE = 20 -- Base lookahead distance
local OBSTACLE_AVOIDANCE_LOOKAHEAD_SPEED_FACTOR = 0.5 -- How much lookahead increases with speed (Lookahead = BASE + SPEED * FACTOR)
local OBSTACLE_AVOIDANCE_RADIUS = 7 -- INCREASED from 6 for wider obstacle detection
local OBSTACLE_SENSOR_COUNT = 36 -- Number of sensor rays to cast (every 10 degrees)
local OBSTACLE_SENSOR_ANGLE_SPAN = 360 -- Full 360-degree sensing for comprehensive detection
local OBSTACLE_WEIGHT = 0.7 -- REDUCED from 0.8 to balance obstacle avoidance vs. goal seeking
local OBSTACLE_SAFETY_DISTANCE = 6 -- INCREASED from 4 for safer distance from obstacles
local OBSTACLE_VISUALIZATION_ENABLED = true -- Set to true to visualize obstacle detection rays
local OBSTACLE_VISUALIZATION_LIFETIME = 0.1 -- How long obstacle visualization rays remain visible
-- VFH algorithm parameters
local VFH_SECTOR_COUNT = 36 -- Number of sectors in the histogram (every 10 degrees)
local VFH_SECTOR_ANGLE = 360 / VFH_SECTOR_COUNT -- Angle of each sector in degrees
local VFH_THRESHOLD = 0.4 -- INCREASED from 0.3 to be less sensitive to minor obstacles
local VFH_SMOOTH_FACTOR = 0.6 -- INCREASED from 0.5 for even smoother histogram transitions
local VFH_MAX_TURNING_RATE = 0.45 -- REDUCED from 0.6 to prevent excessive turning near obstacles
local VFH_HYSTERESIS_FACTOR = 0.85 -- Manus: Added hysteresis to VFH direction selection (lower cost must be < oldCost * factor)
local VFH_FRONTAL_OBSTACLE_PENALTY = 50 -- Manus: Added penalty for blocked frontal sectors to encourage turning away from walls
-- Stuck detection and recovery parameters
local STUCK_THRESHOLD = 18 -- ADJUSTED from 20 to trigger recovery slightly sooner
local STUCK_SPEED_THRESHOLD = 2 -- REDUCED from 3 to only consider truly slow speeds as stuck
local STUCK_MOVEMENT_THRESHOLD = 0.3 -- INCREASED from 0.2 to be less sensitive to small movements
local STUCK_PROGRESS_THRESHOLD = 0.1 -- INCREASED from 0.05 to be less sensitive to small progress
local RECOVERY_DURATION = 25 -- INCREASED from 20 for longer recovery attempts
local REVERSE_THROTTLE = -0.6 -- REDUCED from -0.8 for gentler reverse throttle
local FORCE_REVERSE_THRESHOLD = 15 -- INCREASED from 10 to be more patient before forcing reverse
-- Stability control parameters
local MAX_STABILITY_INCREMENT = 2 -- REDUCED from 3 for more gradual stability changes
local STABILITY_DECAY_RATE = 3 -- INCREASED from 2 for faster stability recovery
local MAX_ANGLE_CHANGE_RATE = 18 -- REDUCED from 20 for smoother angle changes, especially near obstacles
local DIRECTION_SMOOTHING_FACTOR = 0.85 -- INCREASED from 0.8 for smoother direction changes, especially during avoidance
local EMERGENCY_STOP_DISTANCE_MULTIPLIER = 0.5 -- REDUCED from 0.6 for even earlier emergency stops/braking
local MAX_ROTATION_DIFFERENCE = 0.4 -- INCREASED frofile:///C:/Users/Radithya/Downloads/Vibe Coder v0.4.rbxmxm 0.3 for more rotation tolerance
local PREDICTIVE_TURN_DISTANCE_FACTOR = 1.8 -- Manus: Factor of approach distance to start predictive turning
local PREDICTIVE_TURN_ANGLE_THRESHOLD = 30 -- Manus: Angle threshold (degrees) to next waypoint to trigger predictive turning
-- Manus: Continuous Path Recalculation Parameters
local PATH_RECALCULATION_INTERVAL = 1.5 -- Seconds between forced path recalculations
local PATH_DEVIATION_THRESHOLD = 7 -- Studs: Max distance allowed from current path segment before recalc (Increased from 6)
local PATH_WAYPOINT_APPROACH_DISTANCE = 4 -- Studs: How close to get to a path waypoint before advancing
-- Manus: Spin Trap Detection & Escape Parameters
local SPIN_TRAP_THRESHOLD = 4 -- How many consecutive high angle changes trigger spin trap detection (Reduced from 6)
local SPIN_TRAP_STUCK_TIME_THRESHOLD = 30 -- How long stuck before considering spin trap
local SPIN_TRAP_RECOVERY_DURATION = 20 -- Duration of the special spin trap recovery maneuver (longer reverse)
local SPIN_TRAP_REVERSE_THROTTLE = -0.6 -- Stronger reverse for spin trap escape
local SPIN_TRAP_TIMEOUT = 10 -- Seconds: If spin trap recovery doesn't work, force path recalc
-- Manus: Bad Spot Memory Parameters
local BAD_SPOT_MEMORY_ENABLED = true
local BAD_SPOT_RADIUS = 8 -- Studs: How close to a bad spot to consider it relevant
local BAD_SPOT_LIFETIME = 60 -- Seconds: How long a bad spot remains in memory
local BAD_SPOT_VFH_PENALTY = 150 -- Cost penalty added to VFH sectors leading towards a bad spot
local BAD_SPOT_RECORD_STUCK_THRESHOLD = STUCK_THRESHOLD + 10 -- Record spot if stuck longer than this
-- Manus: Dynamic Waypoint Adjustment Parameters
local DYNAMIC_WAYPOINT_ADJUSTMENT_ENABLED = true
local DYNAMIC_WAYPOINT_DISTANCE = 10 -- How far to offset the temporary waypoint
local DYNAMIC_WAYPOINT_RECOVERY_DURATION = 15 -- How many recovery ticks to attempt dynamic waypoint
local PATH_FOLDER = Workspace:WaitForChild("PathNodes", 10)
local WAYPOINTS = {}
-- grab all Vector3Values in one go
local nodeValues = {}
if PATH_FOLDER then
	for _, child in ipairs(PATH_FOLDER:GetChildren()) do
		if child:IsA("Vector3Value") then
			table.insert(nodeValues, child)
		end
	end
else
	warn("NPC Script: PathNodes folder not found!")
end
-- sort by the number at the end of the name (PathNode<number>)
table.sort(nodeValues, function(a, b)
	local aNum = tonumber(a.Name:match("^PathNode(%d+)$")) or math.huge
	local bNum = tonumber(b.Name:match("^PathNode(%d+)$")) or math.huge
	return aNum < bNum
end)
-- collect the Vector3s in order
for _, vecValue in ipairs(nodeValues) do
	table.insert(WAYPOINTS, vecValue.Value)
end
-- debug output
if #WAYPOINTS > 0 then
	print("Collected " .. #WAYPOINTS .. " waypoints:")
	for i, wp in ipairs(WAYPOINTS) do
		print(i, wp) -- e.g. "1 50, 0, 50"
	end
else
	warn("NPC Script: No waypoints collected from PathNodes folder!")
end
local currentWaypointIndex = 1
local AGENT_PARAMETERS = {
	AgentRadius = 3, -- Increased slightly for bike width
	AgentHeight = 5,
	AgentCanJump = false, -- Bike can't jump
	WaypointSpacing = 3 -- Increased slightly
}
local States = {
	IDLE = "Idle",
	FINDING_SEAT = "FindingSeat",
	MOVING_TO_SEAT = "MovingToSeat",
	ATTEMPTING_SIT = "AttemptingSit",
	DRIVING_VEHICLE = "DrivingVehicle",
	EXITING_VEHICLE = "ExitingVehicle",
	MOVING_TO_WAYPOINT = "MovingToWaypoint", -- This state might become unused if driving handles all waypoints
	COMPLETED = "Completed"
}
local currentState = States.IDLE
local targetSeat = nil
local currentThrottle = 0
local targetThrottle = 0
local currentPath = nil -- Path for walking
local currentDrivingPath = nil -- Manus: Path for driving
local currentPathWaypointIndex = 1 -- Manus: Index for currentDrivingPath
local waypointParts = {}
local bikePathParts = {}
local obstacleSensorParts = {}
local lastAvoidanceDirection = nil -- Store previous avoidance direction for smoothing
local lastVFHBestDirection = nil -- Manus: Store previous VFH best direction for hysteresis
local lastVFHBestCost = math.huge -- Manus: Store cost of previous VFH best direction
local dynamicRecoveryTarget = nil -- Manus: Stores the temporary target position for dynamic waypoint adjustment
-- Enhanced debugging system - reduces spam by being more selective
local DEBUG_MODE = true
local LOG_LEVELS = {
	ERROR = 1, -- Always show errors
	WARN = 2, -- Warnings about potential issues
	STATE = 3, -- State transitions only
	INFO = 4, -- Important operational information
	VERBOSE = 5, -- Detailed operational information
	PATH = 6 -- Pathfinding details (very verbose)
}
local CURRENT_LOG_LEVEL = LOG_LEVELS.VERBOSE -- Set this to control verbosity (Increased from INFO for fence debugging)
local function logDebug(level, ...)
	if not DEBUG_MODE or level > CURRENT_LOG_LEVEL then return end
	local prefix = "NPC: "
	if level == LOG_LEVELS.ERROR then prefix = "NPC ERROR: "
	elseif level == LOG_LEVELS.WARN then prefix = "NPC WARNING: "
	elseif level == LOG_LEVELS.STATE then prefix = "NPC STATE: "
	elseif level == LOG_LEVELS.PATH then prefix = "NPC PATH: "
	end
	print(prefix, ...)
end
-- Manus: Bad Spot Memory Table Functions (Defined early for scope, uses string serialization for attributes)
local function deserializeBadSpotMemory(memoryStr)
	local memoryTable = {}
	if memoryStr and #memoryStr > 0 then
		for entry in string.gmatch(memoryStr, "([^\";\"]+)") do
			local parts = {}
			for part in string.gmatch(entry, "([^=]+)") do table.insert(parts, part) end
			if #parts == 2 then
				local posStr = parts[1]
				local timestamp = tonumber(parts[2])
				if timestamp then
					memoryTable[posStr] = timestamp
				end
			end
		end
	end
	return memoryTable
end
local function serializeBadSpotMemory(memoryTable)
	local entries = {}
	for posStr, timestamp in pairs(memoryTable) do
		table.insert(entries, string.format("%s=%.0f", posStr, timestamp))
	end
	return table.concat(entries, ";")
end
local function getBadSpotMemory(bike)
	local memoryStr = bike:GetAttribute("BadSpotMemory") or ""
	local memoryTable = deserializeBadSpotMemory(memoryStr)
	-- Clear expired spots
	local now = tick()
	local updatedMemoryTable = {}
	local changed = false
	for posStr, timestamp in pairs(memoryTable) do
		if now - timestamp < BAD_SPOT_LIFETIME then
			updatedMemoryTable[posStr] = timestamp
		else
			logDebug(LOG_LEVELS.VERBOSE, "Bad spot expired: ", posStr)
			changed = true -- Mark that the table changed due to expiration
		end
	end
	-- Only update attribute if memory changed (expiration or initial load)
	if changed or memoryStr == "" then
		local updatedMemoryStr = serializeBadSpotMemory(updatedMemoryTable)
		bike:SetAttribute("BadSpotMemory", updatedMemoryStr)
	end
	return updatedMemoryTable -- Return the usable table
end
local function recordBadSpot(bike, position)
	if not BAD_SPOT_MEMORY_ENABLED then return end
	local memoryTable = getBadSpotMemory(bike) -- Gets the deserialized table
	-- Use a string representation as table key, rounded to avoid too many unique spots
	local posStr = string.format("%.1f,%.1f,%.1f", position.X, position.Y, position.Z)
	memoryTable[posStr] = tick() -- Update the table
	-- Serialize the updated table and save it back
	local updatedMemoryStr = serializeBadSpotMemory(memoryTable)
	bike:SetAttribute("BadSpotMemory", updatedMemoryStr)
	logDebug(LOG_LEVELS.WARN, "Recorded bad spot at: ", posStr)
end
local function visualizeWaypoint(position, name, color)
	if not WAYPOINT_VISUALIZATION_ENABLED then return end
	local part = Instance.new("Part")
	part.Size = WAYPOINT_PART_SIZE
	part.Position = position + Vector3.new(0, WAYPOINT_PART_SIZE.Y/2, 0) -- Position slightly above ground
	part.Anchored = true
	part.CanCollide = false
	part.Material = Enum.Material.Neon
	part.Color = color or WAYPOINT_PART_COLOR
	part.Name = name or "Waypoint"
	part.Transparency = 0.3
	part.Parent = workspace
	-- Create text label to display waypoint info
	local billboard = Instance.new("BillboardGui")
	billboard.Size = UDim2.new(0, 100, 0, 40)
	billboard.StudsOffset = Vector3.new(0, 1, 0)
	billboard.Adornee = part
	billboard.Parent = part
	local textLabel = Instance.new("TextLabel")
	textLabel.BackgroundTransparency = 1
	textLabel.Size = UDim2.new(1, 0, 1, 0)
	textLabel.Text = name or "Waypoint"
	textLabel.TextColor3 = Color3.new(1, 1, 1)
	textLabel.TextStrokeTransparency = 0
	textLabel.TextStrokeColor3 = Color3.new(0, 0, 0)
	textLabel.TextScaled = true
	textLabel.Parent = billboard
	table.insert(waypointParts, part)
	Debris:AddItem(part, WAYPOINT_LIFETIME)
	return part
end
local function visualizePathway(path, color)
	if not path or not WAYPOINT_VISUALIZATION_ENABLED then return end
	local waypoints = path:GetWaypoints()
	local pathColor = color or Color3.fromRGB(0, 170, 255) -- Blue path default
	-- Create a line to connect all waypoints
	for i = 1, #waypoints - 1 do
		local startPos = waypoints[i].Position
		local endPos = waypoints[i + 1].Position
		-- Create a part to represent the path segment
		local pathPart = Instance.new("Part")
		pathPart.Anchored = true
		pathPart.CanCollide = false
		pathPart.Material = Enum.Material.Neon
		pathPart.Color = pathColor
		pathPart.Transparency = 0.7
		pathPart.Name = "PathSegment_" .. i
		-- Calculate the position and size of the part to represent the path segment
		local direction = endPos - startPos
		local distance = direction.Magnitude
		local midpoint = startPos + direction * 0.5
		-- Create a CFrame that points from start to end
		local cf = CFrame.lookAt(startPos, endPos)
		-- Position the part along the path
		pathPart.Size = Vector3.new(0.2, 0.2, distance)
		pathPart.CFrame = CFrame.new(midpoint, midpoint + cf.LookVector)
		pathPart.Parent = workspace
		table.insert(waypointParts, pathPart)
		Debris:AddItem(pathPart, WAYPOINT_LIFETIME)
	end
	-- Visualize each waypoint
	for i, wp in ipairs(waypoints) do
		visualizeWaypoint(wp.Position, "Path_" .. i, pathColor)
	end
	logDebug(LOG_LEVELS.PATH, "Path visualization created with", #waypoints, "points")
end
-- Function to visualize obstacle detection rays
local function visualizeObstacleSensor(origin, direction, distance, hit, lookahead)
	if not OBSTACLE_VISUALIZATION_ENABLED then return end
	-- Create ray visualization
	local rayPart = Instance.new("Part")
	rayPart.Anchored = true
	rayPart.CanCollide = false
	rayPart.Material = Enum.Material.Neon
	-- Set color based on hit (red if hit, green if clear)
	if hit then
		rayPart.Color = Color3.fromRGB(255, 0, 0) -- Red for hit
	else
		rayPart.Color = Color3.fromRGB(0, 255, 0) -- Green for clear
	end
	rayPart.Transparency = 0.7
	rayPart.Name = "ObstacleSensorRay"
	-- Calculate ray length and position
	local rayLength = hit and distance or lookahead -- Use actual lookahead distance
	-- Set size and position
	rayPart.Size = Vector3.new(0.1, 0.1, rayLength)
	rayPart.CFrame = CFrame.lookAt(origin, origin + direction * rayLength) * CFrame.new(0, 0, -rayLength/2)
	rayPart.Parent = workspace
	table.insert(obstacleSensorParts, rayPart)
	Debris:AddItem(rayPart, OBSTACLE_VISUALIZATION_LIFETIME)
	return rayPart
end
-- New function to visualize the bike's predicted path
local function visualizeBikePath(bikePos, bikeCFrame, targetPos, steerValue, isReversing)
	-- Clear previous bike path visualization
	for _, part in ipairs(bikePathParts) do
		if part and part.Parent then
			part:Destroy()
		end
	end
	bikePathParts = {}
	if not WAYPOINT_VISUALIZATION_ENABLED then return end
	local predictedPoints = {}
	local currentPoint = bikePos
	local currentCFrame = bikeCFrame
	local segmentLength = BIKE_PATH_PREDICTION_DISTANCE / BIKE_PATH_SEGMENTS
	-- Create a direct line to the target first (for reference)
	local directPart = Instance.new("Part")
	directPart.Anchored = true
	directPart.CanCollide = false
	directPart.Material = Enum.Material.Neon
	directPart.Color = Color3.fromRGB(255, 255, 0) -- Yellow for direct path
	directPart.Transparency = 0.8
	directPart.Name = "DirectPath"
	local direction = targetPos - bikePos
	local distance = direction.Magnitude
	local midpoint = bikePos + direction * 0.5
	local cf = CFrame.lookAt(bikePos, targetPos)
	directPart.Size = Vector3.new(0.2, 0.2, distance)
	directPart.CFrame = CFrame.new(midpoint, midpoint + cf.LookVector)
	directPart.Parent = workspace
	table.insert(bikePathParts, directPart)
	Debris:AddItem(directPart, 0.2)
	-- Simple prediction algorithm
	for i = 1, BIKE_PATH_SEGMENTS do
		-- Apply steering to the direction vector
		local forward = currentCFrame.LookVector
		local right = currentCFrame.RightVector
		-- Simulate turning based on steer value
		local turnRate = steerValue * 0.25 -- REDUCED from 0.3 for more gradual turning
		-- If reversing, invert the forward direction
		if isReversing then
			forward = -forward
		end
		local nextDir = (forward + right * turnRate).Unit
		-- Calculate next position
		local nextPoint = currentPoint + nextDir * segmentLength
		table.insert(predictedPoints, nextPoint)
		-- Update for next iteration
		currentPoint = nextPoint
		currentCFrame = CFrame.lookAt(currentPoint, currentPoint + nextDir)
	end
	-- Visualize the predicted path
	for i = 1, #predictedPoints - 1 do
		local startPos = predictedPoints[i]
		local endPos = predictedPoints[i + 1]
		-- Create a part to represent the predicted path segment
		local pathPart = Instance.new("Part")
		pathPart.Anchored = true
		pathPart.CanCollide = false
		pathPart.Material = Enum.Material.Neon
		pathPart.Color = BIKE_PATH_COLOR
		pathPart.Transparency = 0.5
		pathPart.Name = "BikePathSegment_" .. i
		-- Calculate position and size
		local direction = endPos - startPos
		local distance = direction.Magnitude
		local midpoint = startPos + direction * 0.5
		-- Create a CFrame that points from start to end
		local cf = CFrame.lookAt(startPos, endPos)
		-- Position the part along the path
		pathPart.Size = Vector3.new(0.3, 0.3, distance)
		pathPart.CFrame = CFrame.new(midpoint, midpoint + cf.LookVector)
		pathPart.Parent = workspace
		table.insert(bikePathParts, pathPart)
		Debris:AddItem(pathPart, 0.2) -- Short lifetime since we update frequently
	end
end
local function clearWaypointVisualizations()
	for _, part in ipairs(waypointParts) do
		if part and part.Parent then
			part:Destroy()
		end
	end
	waypointParts = {}
	for _, part in ipairs(bikePathParts) do
		if part and part.Parent then
			part:Destroy()
		end
	end
	bikePathParts = {}
	for _, part in ipairs(obstacleSensorParts) do
		if part and part.Parent then
			part:Destroy()
		end
	end
	obstacleSensorParts = {}
end
local function getCurrentDestination()
	if currentWaypointIndex > 0 and currentWaypointIndex <= #WAYPOINTS then
		return WAYPOINTS[currentWaypointIndex]
	else
		logDebug(LOG_LEVELS.WARN, "Invalid currentWaypointIndex:", currentWaypointIndex, "Max:", #WAYPOINTS, ". Returning last waypoint or origin.")
		return #WAYPOINTS > 0 and WAYPOINTS[#WAYPOINTS] or Vector3.new(0,0,0)
	end
end
-- Function to build and smooth the VFH histogram
local function buildVFHistogram(bikePos, bikeCFrame, speed)
	local histogram = {}
	local obstaclePositions = {}
	for i = 1, VFH_SECTOR_COUNT do
		histogram[i] = 0 -- Initialize histogram with zeros (no obstacles)
	end
	if not OBSTACLE_DETECTION_ENABLED then
		return histogram, obstaclePositions
	end
	local currentLookahead = OBSTACLE_AVOIDANCE_LOOKAHEAD_BASE + speed * OBSTACLE_AVOIDANCE_LOOKAHEAD_SPEED_FACTOR
	local raycastParams = RaycastParams.new()
	raycastParams.FilterDescendantsInstances = {NPC, targetSeat and targetSeat:FindFirstAncestorWhichIsA("Model")} -- Ignore NPC and its own bike
	raycastParams.FilterType = Enum.RaycastFilterType.Exclude
	raycastParams.IgnoreWater = true
	-- Cast rays in a circle around the bike
	local angleIncrement = OBSTACLE_SENSOR_ANGLE_SPAN / OBSTACLE_SENSOR_COUNT
	for i = 0, OBSTACLE_SENSOR_COUNT - 1 do
		local angle = math.rad(i * angleIncrement)
		local direction = (CFrame.Angles(0, angle, 0) * bikeCFrame.LookVector).Unit
		local origin = bikePos + Vector3.new(0, 1, 0) -- Start ray slightly above ground
		local result = workspace:Raycast(origin, direction * currentLookahead, raycastParams)
		local hitDistance = currentLookahead
		local hit = false
		if result then
			if result.Instance.Name == "Road Fence" or 
				result.Instance.Name == "ObstacleSensorRay" or 
				result.Instance.Name:match("^PathSegment_") or 
				result.Instance.Name:match("^BikePathSegment_") or 
				result.Instance.Name == "Waypoint" or 
				result.Instance.Name == "DirectPath" then
				logDebug(LOG_LEVELS.VERBOSE, "VFH Ray Hit Ignored (Visualization/Fence): Name=", result.Instance.Name)
			else
				hitDistance = result.Distance
				hit = true
				table.insert(obstaclePositions, result.Position)
				-- Calculate obstacle influence based on distance
				local influence = math.max(0, 1 - (hitDistance / currentLookahead))^2
				-- Determine the sector this obstacle falls into
				local obstacleAngle = math.atan2(direction.Z, direction.X) * (180/math.pi)
				if obstacleAngle < 0 then obstacleAngle = obstacleAngle + 360 end
				local sectorIndex = math.floor(obstacleAngle / VFH_SECTOR_ANGLE) + 1
				if sectorIndex > VFH_SECTOR_COUNT then sectorIndex = 1 end
				-- Add influence to the corresponding sector and adjacent sectors
				for j = -2, 2 do
					local idx = sectorIndex + j
					if idx < 1 then idx = idx + VFH_SECTOR_COUNT end
					if idx > VFH_SECTOR_COUNT then idx = idx - VFH_SECTOR_COUNT end
					local weight = (1 - math.abs(j) / 3)
					histogram[idx] = math.min(1.0, histogram[idx] + influence * weight * OBSTACLE_WEIGHT)
					logDebug(LOG_LEVELS.VERBOSE, "VFH Ray Hit Processed: Name=", result.Instance.Name, ", Class=", result.Instance.ClassName, ", Material=", tostring(result.Instance.Material), ", Pos=", result.Position) -- Log processed hit
				end
			end
		end
		-- Visualize the sensor ray
		visualizeObstacleSensor(origin, direction, hitDistance, hit, currentLookahead)
	end
	-- Smooth the histogram using a moving average
	local smoothedHistogram = {}
	for i = 1, VFH_SECTOR_COUNT do
		local sum = 0
		local count = 0
		for j = -2, 2 do -- Average over 5 sectors
			local idx = i + j
			if idx < 1 then idx = idx + VFH_SECTOR_COUNT end
			if idx > VFH_SECTOR_COUNT then idx = idx - VFH_SECTOR_COUNT end
			-- Weight closer sectors more heavily
			local weight = 1.0 - (math.abs(j) / 3)
			sum = sum + histogram[idx] * weight
			count = count + weight
		end
		smoothedHistogram[i] = sum / count
	end
	return smoothedHistogram, obstaclePositions
end
-- Function to find the best direction using VFH algorithm with improved stability, hysteresis, and frontal penalty
local function findBestDirection(histogram, bikePos, bikeCFrame, targetPos)
	-- If obstacle detection is disabled, just return direction to target
	if not OBSTACLE_DETECTION_ENABLED then
		local dirToTarget = (targetPos - bikePos).Unit
		return dirToTarget
	end
	-- Calculate direction to target
	local dirToTarget = (targetPos - bikePos)
	if dirToTarget.Magnitude < 0.01 then return bikeCFrame.LookVector end -- Avoid issues if already at target
	dirToTarget = Vector3.new(dirToTarget.X, 0, dirToTarget.Z).Unit
	-- Calculate angle to target in degrees (world space)
	local targetAngle = math.atan2(dirToTarget.Z, dirToTarget.X) * (180/math.pi)
	if targetAngle < 0 then targetAngle = targetAngle + 360 end
	-- Calculate current bike heading angle
	local bikeLook = bikeCFrame.LookVector
	local flatBikeLook = Vector3.new(bikeLook.X, 0, bikeLook.Z).Unit
	local currentHeadingAngle = math.atan2(flatBikeLook.Z, flatBikeLook.X) * (180/math.pi)
	if currentHeadingAngle < 0 then currentHeadingAngle = currentHeadingAngle + 360 end
	-- Find candidate directions (sectors with obstacle value below threshold)
	local candidates = {}
	for i = 1, VFH_SECTOR_COUNT do
		if histogram[i] < VFH_THRESHOLD then
			local sectorAngle = (i - 1) * VFH_SECTOR_ANGLE
			local angleDiff = math.abs(sectorAngle - targetAngle)
			if angleDiff > 180 then angleDiff = 360 - angleDiff end
			-- Calculate base cost
			local cost = angleDiff
			if lastVFHBestDirection then
				local lastAngle = math.atan2(lastVFHBestDirection.Z, lastVFHBestDirection.X) * (180/math.pi)
				if lastAngle < 0 then lastAngle = lastAngle + 360 end
				local continuityDiff = math.abs(sectorAngle - lastAngle)
				if continuityDiff > 180 then continuityDiff = 360 - continuityDiff end
				cost = cost + continuityDiff * 0.4 -- Reduced weight for continuity
			end
			-- Manus: Apply Bad Spot Penalty (Calculated *before* inserting into table)
			if BAD_SPOT_MEMORY_ENABLED then
				local bike = NPC:FindFirstAncestorWhichIsA("Model") -- Need bike instance here
				if bike then
					local badSpots = getBadSpotMemory(bike)
					local candidateAngleRad = math.rad(sectorAngle)
					local candidateDir = Vector3.new(math.cos(candidateAngleRad), 0, math.sin(candidateAngleRad)).Unit
					local checkPos = bikePos + candidateDir * (BAD_SPOT_RADIUS / 2) -- Check slightly ahead
					for posStr, _ in pairs(badSpots) do
						local spotPos = Vector3.zero -- Default
						local parts = {}
						for part in string.gmatch(posStr, "[+-]?%d*%.?%d+") do table.insert(parts, tonumber(part)) end
						if #parts == 3 then
							spotPos = Vector3.new(parts[1], parts[2], parts[3])
							local vecToSpot = (spotPos - bikePos)
							local distToSpot = vecToSpot.Magnitude
							local flatVecToSpot = Vector3.new(vecToSpot.X, 0, vecToSpot.Z).Unit
							-- Manus: Directional Check - Only apply penalty if spot is generally in front AND candidate dir points towards it
							local isSpotInFront = flatBikeLook:Dot(flatVecToSpot) > 0.2 -- Spot is within ~78 degrees of front
							local isCandidateTowardsSpot = candidateDir:Dot(flatVecToSpot) > 0.7 -- Candidate dir is within ~45 degrees of spot dir
							if distToSpot < BAD_SPOT_RADIUS and isSpotInFront and isCandidateTowardsSpot then
								local penalty = BAD_SPOT_VFH_PENALTY * (1 - (distToSpot / BAD_SPOT_RADIUS))
								cost = cost + penalty
								logDebug(LOG_LEVELS.VERBOSE, "VFH: Applying *directional* bad spot penalty %.1f to sector %d (Dist: %.1f, InFront: %s, Towards: %s). New Cost: %.1f",
									penalty, i, distToSpot, tostring(isSpotInFront), tostring(isCandidateTowardsSpot), cost)
							end
						end
					end
				end
			end
			-- End Bad Spot Penalty Calculation
			table.insert(candidates, {
				sector = i,
				angle = sectorAngle,
				cost = cost -- Insert the final calculated cost
			})
		end
	end
	-- If no candidates (all directions blocked), find least blocked sector
	if #candidates == 0 then
		logDebug(LOG_LEVELS.WARN, "VFH: All directions blocked, finding least blocked.")
		local minValue = 1.0
		local bestSector = 1
		for i = 1, VFH_SECTOR_COUNT do
			if histogram[i] < minValue then
				minValue = histogram[i]
				bestSector = i
			end
		end
		local bestAngle = (bestSector - 1) * VFH_SECTOR_ANGLE
		local angleDiff = math.abs(bestAngle - targetAngle)
		if angleDiff > 180 then angleDiff = 360 - angleDiff end
		table.insert(candidates, {
			sector = bestSector,
			angle = bestAngle,
			cost = angleDiff + 1000 -- Add high cost penalty
		})
	end
	-- Manus: Add penalty for candidates directly in front if that sector is blocked
	for _, candidate in ipairs(candidates) do
		local angleDiffFromHeading = math.abs(candidate.angle - currentHeadingAngle)
		if angleDiffFromHeading > 180 then angleDiffFromHeading = 360 - angleDiffFromHeading end
		-- Apply penalty if the candidate is within ~20 degrees of the front and the corresponding histogram sector is high
		if angleDiffFromHeading < 20 and histogram[candidate.sector] > VFH_THRESHOLD * 0.5 then
			candidate.cost = candidate.cost + VFH_FRONTAL_OBSTACLE_PENALTY * (histogram[candidate.sector] / VFH_THRESHOLD)
			logDebug(LOG_LEVELS.VERBOSE, "VFH: Applying frontal penalty to sector", candidate.sector, "Cost increased to", candidate.cost)
		end
	end
	-- Sort candidates by cost (including frontal penalty)
	table.sort(candidates, function(a, b) return a.cost < b.cost end)
	-- Select best candidate with Hysteresis
	local bestCandidate = candidates[1]
	local useNewDirection = true
	if lastVFHBestDirection and #candidates > 0 then
		local currentBestCost = bestCandidate.cost
		-- Check if the previously chosen direction is still valid and not significantly worse
		local lastAngle = math.atan2(lastVFHBestDirection.Z, lastVFHBestDirection.X) * (180/math.pi)
		if lastAngle < 0 then lastAngle = lastAngle + 360 end
		local lastSector = math.floor(lastAngle / VFH_SECTOR_ANGLE) + 1
		if lastSector > VFH_SECTOR_COUNT then lastSector = 1 end
		if histogram[lastSector] < VFH_THRESHOLD and lastVFHBestCost < currentBestCost / VFH_HYSTERESIS_FACTOR then
			-- Keep the old direction if it's still clear and not much worse than the new best
			useNewDirection = false
			logDebug(LOG_LEVELS.VERBOSE, "VFH Hysteresis: Keeping previous direction.")
		end
	end
	local bestAngle
	if useNewDirection then
		bestAngle = bestCandidate.angle
		lastVFHBestCost = bestCandidate.cost
	else
		-- Use the angle of the last direction
		bestAngle = math.atan2(lastVFHBestDirection.Z, lastVFHBestDirection.X) * (180/math.pi)
		if bestAngle < 0 then bestAngle = bestAngle + 360 end
		-- Keep the last cost for next frame's hysteresis check
	end
	-- Convert angle to direction vector
	local angleRad = math.rad(bestAngle)
	local bestDirX = math.cos(angleRad)
	local bestDirZ = math.sin(angleRad)
	local bestDir = Vector3.new(bestDirX, 0, bestDirZ).Unit
	-- Store the chosen direction for next frame's smoothing and hysteresis
	lastVFHBestDirection = bestDir
	return bestDir
end
local function findNearestUnoccupiedBikeSeat()
	logDebug(LOG_LEVELS.INFO, "Searching for an unoccupied bike seat within radius", FIND_SEAT_RADIUS)
	local nearestSeat, shortestDist = nil, FIND_SEAT_RADIUS
	for _, desc in ipairs(workspace:GetDescendants()) do
		if desc:IsA("VehicleSeat") and not desc.Occupant then
			local dist = (desc.Position - rootPart.Position).Magnitude
			if dist < shortestDist then
				nearestSeat = desc
				shortestDist = dist
				logDebug(LOG_LEVELS.VERBOSE, "Found potential seat:", desc:GetFullName(), "at distance", dist)
			end
		end
	end
	if nearestSeat then
		logDebug(LOG_LEVELS.INFO, "Found nearest seat at distance", shortestDist)
		visualizeWaypoint(nearestSeat.Position, "BikeTarget")
	else
		logDebug(LOG_LEVELS.WARN, "No unoccupied seat found within radius")
	end
	return nearestSeat
end
-- Manus: Modified to accept start position
local function createPathToPosition(startPosition, targetPosition)
	logDebug(LOG_LEVELS.PATH, "Computing path from", startPosition, "to", targetPosition)
	local path = PathfindingService:CreatePath(AGENT_PARAMETERS)
	local success, errorMessage = pcall(function()
		path:ComputeAsync(startPosition, targetPosition)
	end)
	if not success then
		logDebug(LOG_LEVELS.ERROR, "Path computation failed with error:", errorMessage)
		return nil
	end
	if path.Status == Enum.PathStatus.Success then
		logDebug(LOG_LEVELS.PATH, "Path computed successfully with", #path:GetWaypoints(), "waypoints")
		return path
	else
		logDebug(LOG_LEVELS.WARN, "Pathfinding failed with status:", path.Status, ". Attempting fallback.")
		-- Fallback: Try a shorter path or offset target
		local midPoint = startPosition + (targetPosition - startPosition).Unit * 50  -- Arbitrary midpoint
		local fallbackPath = PathfindingService:CreatePath(AGENT_PARAMETERS)
		fallbackPath:ComputeAsync(startPosition, midPoint)
		if fallbackPath.Status == Enum.PathStatus.Success then
			logDebug(LOG_LEVELS.INFO, "Fallback path to midpoint succeeded.")
			return fallbackPath
		end
		return nil
	end
end
local function followPath(path) -- Only used for walking to seat
	if not path then return false end
	local waypoints = path:GetWaypoints()
	for i, wp in ipairs(waypoints) do
		if wp.Action == Enum.PathWaypointAction.Jump then
			logDebug(LOG_LEVELS.VERBOSE, "Jumping at waypoint", i)
			humanoid.Jump = true
		end
		local waypointPosition = wp.Position
		local waypointName = "WP" .. i .. "/" .. #waypoints
		visualizeWaypoint(waypointPosition, waypointName)
		logDebug(LOG_LEVELS.PATH, "Moving to waypoint", i, "at", waypointPosition)
		humanoid:MoveTo(waypointPosition)
		humanoid.MoveToFinished:Wait() -- Wait until the NPC reaches the waypoint
		logDebug(LOG_LEVELS.PATH, "Reached waypoint", i)
	end
	logDebug(LOG_LEVELS.INFO, "Path following completed")
	return true
end
local function attemptToSit() -- ROBUST VERSION
	logDebug(LOG_LEVELS.INFO, "Attempting to sit in seat")
	if not targetSeat then
		logDebug(LOG_LEVELS.ERROR, "attemptToSit called but targetSeat is nil.")
		return false
	end
	if not targetSeat.Parent then
		logDebug(LOG_LEVELS.WARN, "Seat '" .. targetSeat.Name .. "' is invalid (no Parent) in attemptToSit.")
		targetSeat = nil -- Nil it out as it's confirmed invalid
		return false
	end
	if targetSeat.Occupant == humanoid then
		logDebug(LOG_LEVELS.INFO, "Already seated in the target seat")
		return true
	end
	local distance = (rootPart.Position - targetSeat.Position).Magnitude
	if distance > SEAT_APPROACH_DISTANCE + 2 then -- Added a small buffer
		logDebug(LOG_LEVELS.WARN, "NPC is too far from seat to attempt sit. Distance:", distance)
		return false
	end
	logDebug(LOG_LEVELS.VERBOSE, "NPC moving closer to seat position for sitting:", targetSeat.Position)
	humanoid:MoveTo(targetSeat.Position)
	local success, _ = pcall(function() humanoid.MoveToFinished:Wait(2) end) -- Add a timeout, use pcall
	if not success then
		logDebug(LOG_LEVELS.WARN, "MoveTo targetSeat.Position timed out or failed in attemptToSit.")
		return false
	end
	-- Re-check seat validity after moving closer and waiting
	if not targetSeat or not targetSeat.Parent then
		logDebug(LOG_LEVELS.WARN, "Seat '" .. (targetSeat and targetSeat.Name or "UNKNOWN") .. "' became invalid after moving closer in attemptToSit.")
		targetSeat = nil -- Nil it out as it's confirmed invalid
		return false
	end
	if targetSeat.Occupant and targetSeat.Occupant ~= humanoid then
		logDebug(LOG_LEVELS.WARN, "Seat became occupied by someone else ("..targetSeat.Occupant.Name..") while trying to sit.")
		targetSeat = nil
		return false
	end
	-- Original sitting logic from here
	local originalCanCollide = {}
	for _, part in ipairs(NPC:GetDescendants()) do
		if part:IsA("BasePart") and part ~= rootPart then
			originalCanCollide[part] = part.CanCollide
			part.CanCollide = false
		end
	end
	NPC:SetPrimaryPartCFrame(targetSeat.CFrame * CFrame.new(0, 2, 0))
	task.wait(0.1)
	rootPart.Anchored = true
	targetSeat:Sit(humanoid)
	task.wait(0.3)
	rootPart.Anchored = false
	for part, canCollide in pairs(originalCanCollide) do
		if part and part.Parent then
			part.CanCollide = canCollide
		end
	end
	task.wait(0.2)
	if targetSeat.Occupant == humanoid then
		logDebug(LOG_LEVELS.INFO, "NPC successfully sat in the seat")
		return true
	else
		logDebug(LOG_LEVELS.WARN, "NPC failed to sit in the seat")
		return false
	end
end
-- Manus: Helper function to check path deviation
local function checkPathDeviation(bikePos, path)
	if not path or #path:GetWaypoints() < 2 or currentPathWaypointIndex >= #path:GetWaypoints() then
		return false -- Not enough path points or already at the end
	end
	local pathWaypoints = path:GetWaypoints()
	local segmentStart = pathWaypoints[currentPathWaypointIndex].Position
	local segmentEnd = pathWaypoints[currentPathWaypointIndex + 1].Position
	-- Project bikePos onto the line segment
	local segmentVector = segmentEnd - segmentStart
	local bikeVector = bikePos - segmentStart
	local segmentLengthSq = segmentVector.Magnitude^2
	if segmentLengthSq < 0.01 then return false end -- Avoid division by zero
	local t = bikeVector:Dot(segmentVector) / segmentLengthSq
	t = math.clamp(t, 0, 1) -- Clamp projection to the segment
	local closestPointOnSegment = segmentStart + segmentVector * t
	local deviation = (bikePos - closestPointOnSegment).Magnitude
	if deviation > PATH_DEVIATION_THRESHOLD then
		logDebug(LOG_LEVELS.PATH, "Path deviation detected: %.2f studs (Threshold: %.1f). Triggering recalc.", deviation, PATH_DEVIATION_THRESHOLD)
		return true
	end
	return false
end
-- Enhanced driveBike function with continuous path recalculation, obstacle avoidance, improved stability, anti-overshoot, and anti-spinning
local function driveBike()
	if not targetSeat or targetSeat.Occupant ~= humanoid then
		logDebug(LOG_LEVELS.WARN, "NPC is not seated in target seat during driveBike")
		return false
	end
	local bike = targetSeat:FindFirstAncestorWhichIsA("Model")
	if not bike then
		logDebug(LOG_LEVELS.WARN, "Bike model not detected from seat")
		return false
	end
	local npcThrottle = targetSeat:FindFirstChild("NPC_Throttle")
	local npcSteer = targetSeat:FindFirstChild("NPC_Steer")
	if not npcThrottle or not npcSteer then
		logDebug(LOG_LEVELS.WARN, "NPC control objects missing in seat")
		return false
	end
	local bikePos = (bike.PrimaryPart and bike.PrimaryPart.Position) or targetSeat.Position
	local bikeCFrame = (bike.PrimaryPart and bike.PrimaryPart.CFrame) or targetSeat.CFrame
	local finalDestination = getCurrentDestination() -- The ultimate target for this leg
	if not finalDestination then
		logDebug(LOG_LEVELS.ERROR, "driveBike: Could not get final destination!")
		return true -- Exit if no destination
	end
	-- Get velocity and update movement trackers
	local vel = bike.PrimaryPart and bike.PrimaryPart.Velocity or Vector3.new(0, 0, 0)
	local flatVel = Vector3.new(vel.X, 0, vel.Z)
	local speed = flatVel.Magnitude
	-- Manus: Continuous Path Recalculation Logic
	local now = tick()
	local timeSinceLastRecalc = now - (bike:GetAttribute("LastPathRecalcTime") or 0)
	local needsRecalc = false
	local forceRecalcReason = nil
	-- Reason 1: Interval elapsed
	if timeSinceLastRecalc > PATH_RECALCULATION_INTERVAL then
		forceRecalcReason = "Interval"
		needsRecalc = true
	end
	-- Reason 2: Path is invalid or doesn't exist
	if not needsRecalc and (not currentDrivingPath or currentDrivingPath.Status ~= Enum.PathStatus.Success) then
		forceRecalcReason = "Invalid Path"
		needsRecalc = true
	end
	-- Reason 3: Deviation from path
	if not needsRecalc and currentDrivingPath and checkPathDeviation(bikePos, currentDrivingPath) then
		forceRecalcReason = "Deviation"
		needsRecalc = true -- Log message is inside checkPathDeviation
	end
	-- Reason 4: Current path waypoint index is invalid
	if not needsRecalc and currentDrivingPath and (currentPathWaypointIndex < 1 or currentPathWaypointIndex >= #currentDrivingPath:GetWaypoints()) then
		forceRecalcReason = "Invalid Index"
		needsRecalc = true
	end
	-- Reason 5: Spin Trap Timeout
	local isSpinTrapRecovery = bike:GetAttribute("SpinTrapRecoveryActive") or false
	local spinTrapStartTime = bike:GetAttribute("SpinTrapStartTime") or 0
	if not needsRecalc and isSpinTrapRecovery and (now - spinTrapStartTime) > SPIN_TRAP_TIMEOUT then
		logDebug(LOG_LEVELS.WARN, "Spin trap recovery timed out (%.1fs > %.1fs). Forcing path recalculation.", now - spinTrapStartTime, SPIN_TRAP_TIMEOUT)
		forceRecalcReason = "Spin Timeout"
		needsRecalc = true
		bike:SetAttribute("SpinTrapRecoveryActive", false) -- Exit spin recovery mode after forcing recalc
	end
	-- Perform Recalculation if needed
	if needsRecalc then
		logDebug(LOG_LEVELS.PATH, "Path recalc triggered: %s", forceRecalcReason or "Unknown")
		local newPath = createPathToPosition(bikePos, finalDestination)
		if newPath and #newPath:GetWaypoints() > 0 then
			currentDrivingPath = newPath
			currentPathWaypointIndex = 1 -- Start from the beginning of the new path
			bike:SetAttribute("LastPathRecalcTime", now)
			logDebug(LOG_LEVELS.PATH, "Successfully recalculated path with %d waypoints.", #currentDrivingPath:GetWaypoints())
			visualizePathway(currentDrivingPath, Color3.fromRGB(255, 165, 0)) -- Visualize new path in orange
		else
			logDebug(LOG_LEVELS.WARN, "Path recalculation failed. Will retry.")
			currentDrivingPath = nil -- Invalidate current path
			-- Keep driving towards final destination as fallback
		end
	end
	-- Manus: Dynamic Waypoint Adjustment Logic during Recovery
	local isRecoveryMode = bike:GetAttribute("RecoveryMode") or false
	local recoveryTimer = now - (bike:GetAttribute("RecoveryStartTime") or 0)
	dynamicRecoveryTarget = nil -- Reset by default
	if DYNAMIC_WAYPOINT_ADJUSTMENT_ENABLED and isRecoveryMode and recoveryTimer < DYNAMIC_WAYPOINT_RECOVERY_DURATION then
		logDebug(LOG_LEVELS.INFO, "RECOVERY: Attempting dynamic waypoint adjustment.")
		-- Calculate an offset direction, e.g., perpendicular to current facing or away from obstacle
		local offsetDir
		local isObstacleAvoidanceActive = bike:GetAttribute("ObstacleAvoidanceActive") or false
		local steerValue = npcSteer.Value -- Get current steer value for direction
		if isObstacleAvoidanceActive and lastVFHBestDirection then
			-- Try moving perpendicular to the VFH direction (attempt to sidestep)
			offsetDir = Vector3.new(-lastVFHBestDirection.Z, 0, lastVFHBestDirection.X) * math.sign(steerValue + 0.01) -- Steer determines side
			logDebug(LOG_LEVELS.VERBOSE, "Dynamic WP: Using perpendicular to VFH direction.")
		else
			-- Default: move perpendicular to current facing direction
			offsetDir = Vector3.new(-bikeCFrame.LookVector.Z, 0, bikeCFrame.LookVector.X) * math.sign(steerValue + 0.01)
			logDebug(LOG_LEVELS.VERBOSE, "Dynamic WP: Using perpendicular to current look vector.")
		end
		dynamicRecoveryTarget = bikePos + offsetDir * DYNAMIC_WAYPOINT_DISTANCE
		visualizeWaypoint(dynamicRecoveryTarget, "DynamicWP", Color3.fromRGB(255, 255, 0)) -- Yellow
	end
	-- End Dynamic Waypoint Adjustment Logic
	-- Determine the immediate target position for steering
	local immediateTargetPos
	-- Manus: Use Dynamic Waypoint Target if available during recovery
	if dynamicRecoveryTarget then
		immediateTargetPos = dynamicRecoveryTarget
		logDebug(LOG_LEVELS.INFO, "Using dynamic recovery target: ", immediateTargetPos)
	elseif currentDrivingPath and currentPathWaypointIndex < #currentDrivingPath:GetWaypoints() then
		immediateTargetPos = currentDrivingPath:GetWaypoints()[currentPathWaypointIndex + 1].Position -- Target the *next* point on the path
		-- Visualize immediate target
		if WAYPOINT_VISUALIZATION_ENABLED then
			visualizeWaypoint(immediateTargetPos, "PathWP_"..currentPathWaypointIndex + 1, Color3.fromRGB(255, 0, 0)) -- Red target
		end
	else
		-- If no path or at the end of path, target the final destination
		immediateTargetPos = finalDestination
		logDebug(LOG_LEVELS.PATH, "No valid path or at end of path, targeting final destination.")
	end
	-- Calculate distance and direction to the immediate target
	local vecToTarget = immediateTargetPos - bikePos
	local distToTarget = vecToTarget.Magnitude
	-- Check if we've reached the final destination
	local distToFinalDest = (finalDestination - bikePos).Magnitude
	if distToFinalDest < VEHICLE_DESTINATION_APPROACH_DISTANCE then
		logDebug(LOG_LEVELS.INFO, "Reached final destination waypoint", currentWaypointIndex)
		currentWaypointIndex = currentWaypointIndex + 1
		if currentWaypointIndex > #WAYPOINTS then
			logDebug(LOG_LEVELS.INFO, "All waypoints reached. Exiting vehicle.")
			return true -- Signal to exit vehicle
		else
			logDebug(LOG_LEVELS.INFO, "Moving to next waypoint", currentWaypointIndex)
			finalDestination = getCurrentDestination() -- Update final destination for this leg
			currentDrivingPath = nil -- Force path recalculation for the new leg
			return false -- Continue driving
		end
	end
	-- Check if we've reached the current path waypoint
	if currentDrivingPath and currentPathWaypointIndex < #currentDrivingPath:GetWaypoints() then
		local currentPathWP = currentDrivingPath:GetWaypoints()[currentPathWaypointIndex + 1].Position
		local distToCurrentPathWP = (currentPathWP - bikePos).Magnitude
		if distToCurrentPathWP < PATH_WAYPOINT_APPROACH_DISTANCE then
			logDebug(LOG_LEVELS.PATH, "Reached path waypoint %d/%d", currentPathWaypointIndex + 1, #currentDrivingPath:GetWaypoints())
			currentPathWaypointIndex = currentPathWaypointIndex + 1
			-- No need to recalculate path, just advance the index
		end
	end
	-- Calculate flat vectors for angle calculation
	local flatBikeLook = Vector3.new(bikeCFrame.LookVector.X, 0, bikeCFrame.LookVector.Z).Unit
	local flatDirToTarget = Vector3.new(vecToTarget.X, 0, vecToTarget.Z)
	if flatDirToTarget.Magnitude < 0.01 then
		flatDirToTarget = flatBikeLook -- Avoid issues if already at target
	else
		flatDirToTarget = flatDirToTarget.Unit
	end
	-- Calculate angle difference between bike's forward direction and target direction
	local dot = flatBikeLook:Dot(flatDirToTarget)
	local angleToTarget = math.acos(math.clamp(dot, -1, 1))
	local angleDegrees = math.deg(angleToTarget)
	-- Determine steering direction (left or right)
	local cross = flatBikeLook:Cross(flatDirToTarget)
	local steerDirection = math.sign(cross.Y)
	-- Update movement trackers
	local lastPos = bike:GetAttribute("LastPosition") or bikePos
	local lastAngle = bike:GetAttribute("LastAngle") or angleDegrees
	local movement = (bikePos - lastPos).Magnitude
	local angleChange = math.abs(angleDegrees - lastAngle)
	local angleChangeRate = angleChange / 0.1 -- Approximate rate based on 0.1s wait time
	local progressTowardDest = (lastPos - finalDestination).Magnitude - distToFinalDest
	bike:SetAttribute("LastPosition", bikePos)
	bike:SetAttribute("LastAngle", angleDegrees)
	-- Stuck detection logic
	local stuckTime = bike:GetAttribute("StuckTime") or 0
	local lastStuckTime = stuckTime -- Store previous stuck time for comparison
	local stuckDuration = now - (bike:GetAttribute("StuckStartTime") or 0)
	local isMoving = speed > STUCK_SPEED_THRESHOLD or movement > STUCK_MOVEMENT_THRESHOLD or progressTowardDest > STUCK_PROGRESS_THRESHOLD
	if not isMoving then
		stuckTime = stuckTime + 1
		if stuckTime == 1 then bike:SetAttribute("StuckStartTime", now) end
		bike:SetAttribute("StuckTime", stuckTime)
		-- If stuck time is increasing rapidly, force reverse mode with higher threshold
		if stuckTime > lastStuckTime + 5 and stuckTime > FORCE_REVERSE_THRESHOLD then -- INCREASED from +3 to +5
			bike:SetAttribute("ForceReverseMode", true)
			logDebug(LOG_LEVELS.WARN, "Forcing reverse mode due to rapid stuck counter increase: " .. stuckTime)
		end
		-- Manus: Record bad spot if stuck for too long
		if BAD_SPOT_MEMORY_ENABLED and stuckTime > BAD_SPOT_RECORD_STUCK_THRESHOLD then
			recordBadSpot(bike, bikePos)
			-- Reset stuck time slightly after recording to avoid immediate re-record
			bike:SetAttribute("StuckTime", STUCK_THRESHOLD - 5)
		end
	else
		-- Decrease stuck counter more aggressively when moving well
		stuckTime = math.max(0, stuckTime - 2)
		bike:SetAttribute("StuckTime", stuckTime)
		-- If we're moving well, disable force reverse mode
		if isMoving and progressTowardDest > 0.2 then
			bike:SetAttribute("ForceReverseMode", false)
		end
	end
	-- Rotation stability check (detecting spins)
	local rotationStability = bike:GetAttribute("RotationStability") or 0
	local lastRotation = bike:GetAttribute("LastRotation") or bikeCFrame.LookVector
	local rotationDifference = (bikeCFrame.LookVector - lastRotation).Magnitude
	if rotationDifference > MAX_ROTATION_DIFFERENCE then
		rotationStability = math.min(10, rotationStability + MAX_STABILITY_INCREMENT)
	else
		rotationStability = math.max(0, rotationStability - STABILITY_DECAY_RATE)
	end
	bike:SetAttribute("RotationStability", rotationStability)
	bike:SetAttribute("LastRotation", bikeCFrame.LookVector)
	-- Manus: Spin Trap Detection (High angle change rate)
	local highAngleChangeCount = bike:GetAttribute("HighAngleChangeCount") or 0
	if angleChangeRate > MAX_ANGLE_CHANGE_RATE and angleDegrees > 45 then -- Only count significant changes at larger angles
		highAngleChangeCount = highAngleChangeCount + 1
		logDebug(LOG_LEVELS.VERBOSE, "High angle change detected (%d). Count: %d", angleChangeRate, highAngleChangeCount)
	else
		-- Reset counter if angle change is low or angle is small
		highAngleChangeCount = math.max(0, highAngleChangeCount - 1)
	end
	bike:SetAttribute("HighAngleChangeCount", highAngleChangeCount)
	-- Trigger Spin Trap Recovery if threshold met and stuck
	local isSpinTrapRecovery = bike:GetAttribute("SpinTrapRecoveryActive") or false
	if not isSpinTrapRecovery and highAngleChangeCount > SPIN_TRAP_THRESHOLD and stuckTime > SPIN_TRAP_STUCK_TIME_THRESHOLD then
		logDebug(LOG_LEVELS.WARN, "Spin Trap Detected! (Angle Changes: %d, Stuck: %d). Initiating recovery.", highAngleChangeCount, stuckTime)
		bike:SetAttribute("SpinTrapRecoveryActive", true)
		bike:SetAttribute("SpinTrapStartTime", now)
		isSpinTrapRecovery = true
		-- Immediately enter recovery mode as well to utilize its base logic
		if not isRecoveryMode then
			bike:SetAttribute("RecoveryMode", true)
			bike:SetAttribute("RecoveryStartTime", now)
			isRecoveryMode = true
		end
		needsRecalc = true  -- Force recalc on spin detection
		forceRecalcReason = "Spin Trap"
	end
	-- Recovery mode logic
	local forceReverseMode = bike:GetAttribute("ForceReverseMode") or false
	if not isRecoveryMode and (bike:GetAttribute("StuckTime") or 0) > STUCK_THRESHOLD then
		logDebug(LOG_LEVELS.WARN, "Entering recovery mode, stuck time: %.1f, rotation stability: %d, force reverse: %s", stuckDuration, rotationStability, tostring(forceReverseMode))
		bike:SetAttribute("RecoveryMode", true)
		bike:SetAttribute("RecoveryStartTime", now)
		isRecoveryMode = true
	elseif isRecoveryMode and stuckTime == 0 and recoveryTimer > 1 then -- Exit recovery only if unstuck and after at least 1 second
		logDebug(LOG_LEVELS.INFO, "Exiting recovery mode")
		bike:SetAttribute("RecoveryMode", false)
		bike:SetAttribute("ForceReverseMode", false) -- Ensure force reverse is off
		bike:SetAttribute("ReverseCount", 0) -- Reset reverse count
		isRecoveryMode = false
	elseif isRecoveryMode and recoveryTimer > RECOVERY_DURATION then
		logDebug(LOG_LEVELS.WARN, "Recovery mode timed out (%.1fs). Exiting recovery.", recoveryTimer)
		bike:SetAttribute("RecoveryMode", false)
		bike:SetAttribute("ForceReverseMode", false)
		bike:SetAttribute("ReverseCount", 0)
		isRecoveryMode = false
		-- Force path recalculation after recovery timeout
		currentDrivingPath = nil
	end
	-- Obstacle Avoidance (VFH)
	local smoothedHistogram, obstaclePositions = buildVFHistogram(bikePos, bikeCFrame, speed)
	local avoidanceDirection = findBestDirection(smoothedHistogram, bikePos, bikeCFrame, immediateTargetPos)
	local isObstacleAvoidanceActive = false
	local vecToAvoidanceDir = avoidanceDirection - flatBikeLook
	if OBSTACLE_DETECTION_ENABLED and vecToAvoidanceDir.Magnitude > 0.1 then -- Check if VFH suggests a different direction
		isObstacleAvoidanceActive = true
		-- Smooth the transition between goal seeking and obstacle avoidance
		local weight = math.min(1.0, #obstaclePositions * 0.1) -- Weight avoidance more if more obstacles detected
		flatDirToTarget = (flatDirToTarget * (1 - weight) + avoidanceDirection * weight).Unit
		logDebug(LOG_LEVELS.VERBOSE, "Obstacle avoidance active. Blended target direction.")
	end
	bike:SetAttribute("ObstacleAvoidanceActive", isObstacleAvoidanceActive)
	-- Recalculate angle and steer direction based on potentially modified target direction
	dot = flatBikeLook:Dot(flatDirToTarget)
	angleToTarget = math.acos(math.clamp(dot, -1, 1))
	angleDegrees = math.deg(angleToTarget)
	cross = flatBikeLook:Cross(flatDirToTarget)
	steerDirection = math.sign(cross.Y)
	-- If spin trap recovery is active, update its timer
	if isSpinTrapRecovery then
		local spinRecoveryTimer = now - spinTrapStartTime
		if spinRecoveryTimer > SPIN_TRAP_RECOVERY_DURATION then
			logDebug(LOG_LEVELS.WARN, "Spin trap recovery finished (%.1fs). Returning to normal operation.", spinRecoveryTimer)
			bike:SetAttribute("SpinTrapRecoveryActive", false)
			bike:SetAttribute("HighAngleChangeCount", 0) -- Reset spin counter after recovery
			bike:SetAttribute("StuckTime", 0) -- Reset stuck time
			isSpinTrapRecovery = false
		else
			-- Continue spin trap recovery logic
			logDebug(LOG_LEVELS.INFO, "Spin Trap Recovery Active (%.1fs / %.1fs)", spinRecoveryTimer, SPIN_TRAP_RECOVERY_DURATION)
		end
	end
	-- Determine if we're in alignment phase or movement phase
	local isAlignmentPhase = bike:GetAttribute("AlignmentPhase") or false
	local alignmentStartTime = bike:GetAttribute("AlignmentStartTime") or 0
	-- State transition logic for alignment vs. movement with improved stability
	-- Manus: Prevent entering alignment during spin trap recovery
	if not isSpinTrapRecovery then
		if not isAlignmentPhase and angleDegrees > 45 then
			-- Enter alignment phase when angle is large
			isAlignmentPhase = true
			alignmentStartTime = now
			logDebug(LOG_LEVELS.INFO, "Entering alignment phase, angle: " .. angleDegrees)
		elseif isAlignmentPhase and angleDegrees < 20 and (now - alignmentStartTime) > 0.5 then -- INCREASED from 0.3 for more stable transitions
			-- Exit alignment phase when angle is small and we've been aligning for at least 0.5 seconds
			isAlignmentPhase = false
			logDebug(LOG_LEVELS.INFO, "Exiting alignment phase, angle: " .. angleDegrees)
		end
	end
	bike:SetAttribute("AlignmentPhase", isAlignmentPhase)
	bike:SetAttribute("AlignmentStartTime", alignmentStartTime)
	-- Determine throttle and steering based on phase and angle
	local throttle = 0
	local steerValue = 0
	-- Manus: Prioritize Spin Trap Recovery Maneuver
	if isSpinTrapRecovery then
		logDebug(LOG_LEVELS.WARN, "SPIN TRAP RECOVERY: Executing escape maneuver.")
		throttle = SPIN_TRAP_REVERSE_THROTTLE -- Strong reverse
		-- Steer away from the likely obstacle direction (opposite of current target direction)
		steerValue = -steerDirection * 0.8 -- Strong steering away
		-- Reset PID integral during spin recovery
		bike:SetAttribute("SteeringIntegral", 0)
	elseif isRecoveryMode then
		-- Enhanced recovery mode logic with more reliable reversing
		local recoveryPhase = math.floor(recoveryTimer / 15) % 4
		-- Force reverse for the first half of recovery time OR if dynamic waypoint is NOT active
		if recoveryTimer > RECOVERY_DURATION / 2 or forceReverseMode or not dynamicRecoveryTarget then
			throttle = REVERSE_THROTTLE -- Strong reverse throttle
			steerValue = steerDirection * 0.6 -- REDUCED from 0.8 for more stable steering
			-- If we're making no progress in reverse, try alternating steering
			if stuckTime > lastStuckTime and stuckDuration > 2 then
				steerValue = -steerValue * 0.7 -- Flip steering direction with reduced intensity
				logDebug(LOG_LEVELS.WARN, "Alternating steering direction in reverse due to continued stuck state")
			end
			logDebug(LOG_LEVELS.INFO, "RECOVERY: Reversing with throttle " .. throttle .. " and steer " .. steerValue)
		else
			-- If dynamic waypoint is active (first part of recovery), move towards it gently
			logDebug(LOG_LEVELS.INFO, "RECOVERY: Moving towards dynamic waypoint.")
			throttle = 0.4 -- Gentle forward throttle
			-- Steering is handled by the main PID logic targeting the dynamic waypoint
			-- Use PID logic below, but maybe with adjusted gains for recovery?
			-- For now, let the standard movement PID handle steering towards dynamic target
			local proportionalGain = 0.7
			local integralGain = 0.04
			local derivativeGain = 0.2
			local steeringIntegral = bike:GetAttribute("SteeringIntegral") or 0
			steeringIntegral = math.clamp(steeringIntegral + steerDirection * 0.015, -0.35, 0.35)
			bike:SetAttribute("SteeringIntegral", steeringIntegral)
			local proportionalTerm = steerDirection * proportionalGain
			local integralTerm = steeringIntegral * integralGain
			local derivativeTerm = angleChangeRate * steerDirection * derivativeGain
			steerValue = proportionalTerm + integralTerm - derivativeTerm
		end
		-- Increment reverse count to track recovery attempts (even if moving forward to dynamic WP)
		bike:SetAttribute("ReverseCount", (bike:GetAttribute("ReverseCount") or 0) + 1)
	elseif isAlignmentPhase then
		-- Alignment phase - focus on turning to face target with minimal forward movement
		-- Improved PID controller for alignment with reduced gains for stability
		local proportionalGain = 0.9 -- REDUCED from 1.2 for more stable turning
		local integralGain = 0.03 -- REDUCED from 0.05 for more stable turning
		local derivativeGain = 0.1 -- INCREASED from 0.08 for better damping
		local steeringIntegral = bike:GetAttribute("SteeringIntegral") or 0
		if math.abs(steerDirection) < 0.3 then
			steeringIntegral = math.clamp(steeringIntegral + steerDirection * 0.01, -0.3, 0.3) -- REDUCED from 0.02/-0.4/0.4
		else
			steeringIntegral = steeringIntegral * 0.9 -- INCREASED from 0.85 for more stable response
		end
		bike:SetAttribute("SteeringIntegral", steeringIntegral)
		local proportionalTerm = steerDirection * proportionalGain
		local integralTerm = steeringIntegral * integralGain
		-- Improved derivative calculation using actual angle change
		local derivativeTerm = angleChangeRate * steerDirection * derivativeGain
		local pidSteer = proportionalTerm + integralTerm - derivativeTerm
		-- During alignment, use minimal throttle and focus on steering
		if angleDegrees > 90 then
			throttle = 0 -- Stop completely for very large angles
			steerValue = steerDirection * math.min(0.8, math.abs(pidSteer)) -- REDUCED from 1.0/1.5
		else
			-- Reduce throttle further when obstacle avoidance is active
			throttle = isObstacleAvoidanceActive and 0.05 or 0.1
			steerValue = steerDirection * math.min(0.8, math.abs(pidSteer)) -- REDUCED from 1.0
		end
	else
		-- Movement phase - focus on moving toward target with appropriate steering
		-- Standard PID controller for normal driving with anti-overshoot modifications
		local proportionalGain = 0.6 -- REDUCED from 0.7 for more stable turning
		local integralGain = 0.03 -- REDUCED from 0.05 for less accumulated correction
		local derivativeGain = 0.25 -- INCREASED from 0.15 for stronger dampening of rapid changes
		local steeringIntegral = bike:GetAttribute("SteeringIntegral") or 0
		if math.abs(steerDirection) < 0.3 then
			steeringIntegral = math.clamp(steeringIntegral + steerDirection * 0.01, -0.3, 0.3) -- REDUCED from 0.02
		else
			steeringIntegral = steeringIntegral * 0.9 -- Faster decay to prevent integral buildup
		end
		bike:SetAttribute("SteeringIntegral", steeringIntegral)
		local proportionalTerm = steerDirection * proportionalGain
		local integralTerm = steeringIntegral * integralGain
		-- Enhanced derivative calculation using actual angle change rate
		local derivativeTerm = angleChangeRate * steerDirection * derivativeGain
		-- Add anti-overshoot component that increases dampening as we approach target angle = 0
		local approachFactor = 1 - math.max(0, math.cos(angleToTarget))^2
		local antiOvershootDamping = steerDirection * approachFactor * 0.4
		-- Calculate final steering with anti-overshoot correction
		local pidSteer = proportionalTerm + integralTerm - derivativeTerm - antiOvershootDamping
		local obstacleSpeedFactor = isObstacleAvoidanceActive and 0.6 or 1.0 -- REDUCED from 0.7 for more caution
		-- Manus: Predictive Turning Logic (using final destination for prediction)
		local predictiveTurnThrottleFactor = 1.0
		local predictiveTurnSteerFactor = 1.0
		if currentWaypointIndex < #WAYPOINTS and distToFinalDest < VEHICLE_DESTINATION_APPROACH_DISTANCE * PREDICTIVE_TURN_DISTANCE_FACTOR then
			local nextWp = WAYPOINTS[currentWaypointIndex + 1]
			local vecToNextWp = Vector3.new(nextWp.X - bikePos.X, 0, nextWp.Z - bikePos.Z).Unit
			local dotNext = flatBikeLook:Dot(vecToNextWp)
			local angleToNextWp = math.acos(math.clamp(dotNext, -1, 1)) * (180/math.pi)
			if angleToNextWp > PREDICTIVE_TURN_ANGLE_THRESHOLD then
				logDebug(LOG_LEVELS.INFO, "Predictive turn: Angle to next WP > "..PREDICTIVE_TURN_ANGLE_THRESHOLD.." degrees. Slowing down.")
				-- Reduce speed more significantly when needing to turn sharply for the next waypoint
				predictiveTurnThrottleFactor = math.max(0.2, 1.0 - (angleToNextWp / 90.0) * 0.7)
				-- Could potentially adjust steering towards next waypoint here too, but let's start with throttle
			end
		end
		-- Progressive steering reduction as we approach the target angle
		if angleDegrees < 10 then
			-- Gentler steering for fine adjustments to prevent oscillation
			throttle = 1.0 * obstacleSpeedFactor * predictiveTurnThrottleFactor
			steerValue = steerDirection * math.pow(math.abs(pidSteer * 0.5), 1.3)
		elseif angleDegrees < 20 then
			throttle = 0.95 * obstacleSpeedFactor * predictiveTurnThrottleFactor
			steerValue = steerDirection * math.pow(math.abs(pidSteer * 0.6), 1.2)
		elseif angleDegrees < 45 then
			throttle = 0.85 * obstacleSpeedFactor * predictiveTurnThrottleFactor
			steerValue = pidSteer * 0.65
		else
			throttle = 0.7 * obstacleSpeedFactor * predictiveTurnThrottleFactor
			steerValue = steerDirection * math.min(0.7, math.abs(pidSteer * 0.8))
		end
		-- Enhanced special case for reversing when needed
		if stuckTime > FORCE_REVERSE_THRESHOLD and speed < 2 then
			-- If we're stuck and barely moving, try reversing
			throttle = REVERSE_THROTTLE
			steerValue = -steerDirection * 0.6 -- REDUCED from 0.8
			bike:SetAttribute("ReverseCount", (bike:GetAttribute("ReverseCount") or 0) + 1)
			logDebug(LOG_LEVELS.WARN, "Initiating emergency reverse due to stuck state: " .. stuckTime)
		elseif (bike:GetAttribute("ReverseCount") or 0) > 0 and (bike:GetAttribute("ReverseCount") or 0) < 20 and angleDegrees > 135 then
			throttle = REVERSE_THROTTLE
			steerValue = -steerDirection * 0.6 -- REDUCED from 0.8
			bike:SetAttribute("ReverseCount", bike:GetAttribute("ReverseCount") + 1)
		elseif progressTowardDest > 0.5 then
			bike:SetAttribute("ReverseCount", 0)
		end
	end
	-- Improved oscillation detection and dampening
	local lastSteeringValueAttr = bike:GetAttribute("LastSteeringValue") or 0
	if math.sign(steerValue) ~= math.sign(lastSteeringValueAttr) and math.abs(steerValue) > 0.2 then -- REDUCED from 0.3
		local oscillations = bike:GetAttribute("ConsecutiveOscillations") or 0
		bike:SetAttribute("ConsecutiveOscillations", oscillations + 1)
		-- Progressive dampening based on oscillation count
		if oscillations > 1 then -- REDUCED from 2 for earlier dampening
			local dampFactor = math.max(0.5, 1.0 - (oscillations * 0.1)) -- INCREASED from 0.4/0.08
			steerValue = steerValue * dampFactor
			throttle = throttle * dampFactor
			if oscillations > 4 then -- REDUCED from 6 for faster reset
				bike:SetAttribute("ConsecutiveOscillations", 0)
			end
		end
	elseif math.abs(steerValue) < 0.1 then -- REDUCED from 0.2
		bike:SetAttribute("ConsecutiveOscillations", 0)
	end
	-- Slow down when approaching final destination or when obstacles are nearby
	if distToFinalDest < VEHICLE_DESTINATION_APPROACH_DISTANCE * 2 then
		throttle = throttle * 0.6
	end
	-- Additional slowdown for very close obstacles
	if isObstacleAvoidanceActive then
		local currentLookahead = OBSTACLE_AVOIDANCE_LOOKAHEAD_BASE + speed * OBSTACLE_AVOIDANCE_LOOKAHEAD_SPEED_FACTOR
		local closestObstacleDistance = currentLookahead -- Use current lookahead
		local frontalObstacle = false
		for _, obstaclePos in ipairs(obstaclePositions) do
			local dist = (obstaclePos - bikePos).Magnitude
			if dist < closestObstacleDistance then
				closestObstacleDistance = dist
				-- Check if this closest obstacle is roughly in front
				local vecToObstacle = (obstaclePos - bikePos).Unit
				if flatBikeLook:Dot(vecToObstacle) > 0.8 then -- Within ~37 degrees of front
					frontalObstacle = true
				end
			end
		end
		if closestObstacleDistance < OBSTACLE_SAFETY_DISTANCE * 2 then
			local slowFactor = math.max(0.3, closestObstacleDistance / (OBSTACLE_SAFETY_DISTANCE * 2))
			throttle = throttle * slowFactor
			-- Emergency stop/brake if obstacle is extremely close, especially if frontal
			if closestObstacleDistance < OBSTACLE_SAFETY_DISTANCE * EMERGENCY_STOP_DISTANCE_MULTIPLIER then
				-- Apply graduated braking based on distance, more aggressive if frontal
				local brakeIntensity = math.min(1.0, OBSTACLE_SAFETY_DISTANCE / (closestObstacleDistance + 0.1))
				local brakeThrottle = -0.3 * brakeIntensity -- Base brake
				if frontalObstacle then
					brakeThrottle = -0.6 * brakeIntensity -- Stronger brake if frontal
					logDebug(LOG_LEVELS.WARN, "Emergency BRAKE - FRONTAL obstacle extremely close: %.2f", closestObstacleDistance)
				else
					logDebug(LOG_LEVELS.WARN, "Emergency stop - obstacle extremely close: %.2f", closestObstacleDistance)
				end
				throttle = brakeThrottle
			end
		end
	end
	bike:SetAttribute("LastSteeringValue", steerValue)
	-- Improved steering rate limiting based on speed and angle
	local lastSteerApplied = npcSteer.Value
	local steerChangeRate = bike:GetAttribute("SteeringChangeRate") or 0.15 -- REDUCED from 0.2
	-- Adaptive steering rate based on speed, alignment needs, and angle
	local maxSteerChange
	if isAlignmentPhase then
		maxSteerChange = steerChangeRate * 0.5 -- REDUCED from 0.8 for smoother steering changes
	else
		-- Make steering more responsive at small angles to prevent drift
		if angleDegrees < 15 then
			maxSteerChange = steerChangeRate * (speed < 5 and 0.4 or 0.3) -- REDUCED from 0.5/0.4
		else
			maxSteerChange = steerChangeRate * (speed < 5 and 0.3 or 0.15) -- REDUCED from 0.4/0.2
		end
	end
	-- Increase steering responsiveness during obstacle avoidance or recovery
	-- Manus: Reduce steering rate limit if spinning is detected (high angle change count)
	local spinDampFactor = 1.0
	if highAngleChangeCount > 3 then
		logDebug(LOG_LEVELS.WARN, "Anti-Spin: High angle change detected ("..highAngleChangeCount.."), dampening steering rate.")
		spinDampFactor = math.max(0.3, 1.0 - (highAngleChangeCount - 3) * 0.15)
		maxSteerChange = maxSteerChange * spinDampFactor
	elseif isObstacleAvoidanceActive then
		maxSteerChange = maxSteerChange * 1.5 -- REDUCED from 1.8
	end
	if isRecoveryMode or isSpinTrapRecovery then -- Apply increased rate during any recovery
		maxSteerChange = maxSteerChange * 1.5 -- REDUCED from 2.0
	end
	local steerDelta = steerValue - lastSteerApplied
	if math.abs(steerDelta) > maxSteerChange then
		steerValue = lastSteerApplied + (maxSteerChange * math.sign(steerDelta))
	end
	-- Speed-based steering adjustment with improved small angle handling
	if speed > 15 then
		local speedFactor = math.min(1.0, 25 / (speed + 10))
		-- Ensure minimum steering at high speeds for small angles
		if angleDegrees < 15 and math.abs(steerValue) > 0 then
			steerValue = math.sign(steerValue) * math.max(math.abs(steerValue * speedFactor), 0.015) -- REDUCED from 0.02
		else
			steerValue = steerValue * speedFactor
		end
	end
	steerValue = math.clamp(steerValue, -1, 1)
	-- Visualization and logging
	if WAYPOINT_VISUALIZATION_ENABLED then
		local recoveryStatus = "NORMAL"
		if isSpinTrapRecovery then recoveryStatus = "SPIN_RECOVERY" elseif isRecoveryMode then recoveryStatus = "RECOVERY" end
		local status = string.format(
			"Speed: %.1f | Dist: %.0f | Angle: %.0f° | Steer: %.2f | Stuck: %d | Stability: %d | %s | %s | Obstacles: %s | SpinCnt: %d | PathWP: %d/%d",
			speed, distToFinalDest, angleDegrees, steerValue,
			bike:GetAttribute("StuckTime"), bike:GetAttribute("RotationStability"),
			recoveryStatus,
			isAlignmentPhase and "ALIGNING" or "MOVING",
			isObstacleAvoidanceActive and "YES" or "NO",
			highAngleChangeCount, -- Manus: Added spin counter to log
			currentPathWaypointIndex, currentDrivingPath and #currentDrivingPath:GetWaypoints() or 0 -- Manus: Added path waypoint info
		)
		if bike:GetAttribute("StuckTime") % 15 == 0 or isRecoveryMode or isSpinTrapRecovery or highAngleChangeCount > 0 then logDebug(LOG_LEVELS.INFO, status) end
		visualizeBikePath(bikePos, bikeCFrame, immediateTargetPos, steerValue, throttle < 0)
	end
	-- Apply controls to the bike
	npcThrottle.Value = throttle
	targetSeat.ThrottleFloat = throttle
	npcSteer.Value = steerValue
	targetSeat.SteerFloat = steerValue
	-- Apply torque for turning
	local torqueStrength = 3500 * steerValue * spinDampFactor -- REDUCED from 4000, Manus: Added spin dampening
	if targetSeat:FindFirstChild("Torque") then
		targetSeat.Torque.Value = Vector3.new(0, torqueStrength, 0)
	elseif targetSeat:FindFirstChild("TurnTorque") then
		targetSeat.TurnTorque.Value = torqueStrength
	else
		local bikeModel = targetSeat:FindFirstAncestorWhichIsA("Model")
		if bikeModel and bikeModel.PrimaryPart then
			bikeModel.PrimaryPart:ApplyAngularImpulse(Vector3.new(0, torqueStrength * 0.6, 0)) -- REDUCED from 0.7
		end
	end
	-- Additional logging
	if WAYPOINT_VISUALIZATION_ENABLED and (bike:GetAttribute("StuckTime") % 15 == 0 or isRecoveryMode or isSpinTrapRecovery) then
		logDebug(LOG_LEVELS.VERBOSE, "Applied steering: " .. steerValue ..
			" | Applied torque: " .. torqueStrength ..
			" | Throttle: " .. throttle)
	end
	-- Safety exit if permanently stuck
	if bike:GetAttribute("StuckTime") > 300 then
		logDebug(LOG_LEVELS.WARN, "Vehicle is permanently stuck. Exiting vehicle.")
		return true
	end
	return false
end
local function exitVehicle()
	logDebug(LOG_LEVELS.INFO, "Exiting vehicle process started")
	if humanoid.Sit and humanoid.SeatPart then
		local currentSeat = humanoid.SeatPart -- Store current seat
		local npcThrottle = currentSeat:FindFirstChild("NPC_Throttle") -- Use currentSeat
		local npcSteer = currentSeat:FindFirstChild("NPC_Steer") -- Use currentSeat
		if npcThrottle then npcThrottle.Value = 0 end
		if npcSteer then npcSteer.Value = 0 end
		humanoid.Jump = true
		task.wait(0.1) -- Allow jump to register
		humanoid.Sit = false -- Explicitly set Sit to false
		local exitTimeout = tick() + 2 -- 2 seconds to try
		while humanoid.Sit and humanoid.SeatPart == currentSeat and tick() < exitTimeout do
			humanoid.Jump = true -- Keep trying jump if still seated in the same seat
			task.wait(0.2)
		end
		if not humanoid.Sit or humanoid.SeatPart ~= currentSeat then
			logDebug(LOG_LEVELS.INFO, "NPC successfully exited the vehicle.")
		else
			logDebug(LOG_LEVELS.WARN, "NPC failed to exit vehicle via jump/Sit=false. Forcing SeatPart = nil.")
			humanoid.SeatPart = nil -- Force it if necessary
		end
		targetSeat = nil -- Clear targetSeat regardless
		return true
	end
	humanoid.Sit = false -- Ensure Sit is false even if not in a seat part
	targetSeat = nil
	logDebug(LOG_LEVELS.INFO, "NPC exitVehicle: NPC was not seated or targetSeat was nil.")
	return true
end
local function setState(newState)
	if currentState == newState then return end
	logDebug(LOG_LEVELS.STATE, "Transitioning from", currentState, "to", newState)
	currentState = newState
end
local function updateStateMachine()
	if humanoid.Health <= 0 then
		logDebug(LOG_LEVELS.WARN, "NPC died. Stopping state machine.")
		return true -- Signal to stop the loop
	end
	if humanoid.Sit and humanoid.SeatPart and currentState ~= States.DRIVING_VEHICLE then
		targetSeat = humanoid.SeatPart
		logDebug(LOG_LEVELS.INFO, "Detected NPC seated in vehicle, ensuring state is DRIVING_VEHICLE.")
		setState(States.DRIVING_VEHICLE)
		-- Let DRIVING_VEHICLE logic run in the same frame
	end
	if currentState == States.IDLE then
		if #WAYPOINTS == 0 then
			logDebug(LOG_LEVELS.WARN, "No waypoints defined. Staying IDLE.")
			task.wait(5)
			return false
		end
		visualizeWaypoint(getCurrentDestination(), "InitialDest")
		setState(States.FINDING_SEAT)
	elseif currentState == States.FINDING_SEAT then -- ROBUST VERSION
		local found = findNearestUnoccupiedBikeSeat()
		if found and found.Parent then -- Ensure it's still part of the workspace
			targetSeat = found
			logDebug(LOG_LEVELS.INFO, "Found valid seat:", targetSeat:GetFullName(), ". Transitioning to MOVING_TO_SEAT.")
			setState(States.MOVING_TO_SEAT)
		else
			if found and not found.Parent then
				logDebug(LOG_LEVELS.WARN, "Found seat was already invalid (no parent). Re-finding shortly.")
			else
				logDebug(LOG_LEVELS.INFO, "No suitable seat found. Waiting and retrying FINDING_SEAT.")
			end
			targetSeat = nil -- Ensure targetSeat is nil if no valid seat found
			task.wait(1.5) -- Wait a bit longer before retrying
			-- No explicit setState, will retry FINDING_SEAT in the next tick due to currentState not changing
		end
	elseif currentState == States.MOVING_TO_SEAT then -- ROBUST VERSION
		if not targetSeat or not targetSeat.Parent then
			logDebug(LOG_LEVELS.WARN, "Target seat became invalid (pre-path validation) in MOVING_TO_SEAT. Re-finding.")
			targetSeat = nil
			setState(States.FINDING_SEAT)
			return false
		end
		local distance = (rootPart.Position - targetSeat.Position).Magnitude
		if distance <= SEAT_APPROACH_DISTANCE then
			logDebug(LOG_LEVELS.INFO, "Close enough to seat. Transitioning to ATTEMPTING_SIT.")
			setState(States.ATTEMPTING_SIT)
		else
			logDebug(LOG_LEVELS.INFO, "Creating path to seat at distance", distance)
			local path = createPathToPosition(rootPart.Position, targetSeat.Position) -- Use rootPart pos
			if path then
				visualizePathway(path)
				followPath(path)
				-- After following path, check if we're close enough to the seat
				if targetSeat and targetSeat.Parent then -- Re-validate seat after path following
					local newDistance = (rootPart.Position - targetSeat.Position).Magnitude
					if newDistance <= SEAT_APPROACH_DISTANCE then
						logDebug(LOG_LEVELS.INFO, "Path following complete, close enough to seat. Transitioning to ATTEMPTING_SIT.")
						setState(States.ATTEMPTING_SIT)
					else
						logDebug(LOG_LEVELS.WARN, "Path following complete but still not close enough to seat. Distance:", newDistance)
						-- Stay in MOVING_TO_SEAT, will try again next tick
					end
				else
					logDebug(LOG_LEVELS.WARN, "Seat became invalid during path following. Re-finding.")
					targetSeat = nil
					setState(States.FINDING_SEAT)
				end
			else
				logDebug(LOG_LEVELS.WARN, "Failed to create path to seat. Moving directly.")
				humanoid:MoveTo(targetSeat.Position)
				humanoid.MoveToFinished:Wait()
				-- After direct movement, check if we're close enough
				if targetSeat and targetSeat.Parent then
					local newDistance = (rootPart.Position - targetSeat.Position).Magnitude
					if newDistance <= SEAT_APPROACH_DISTANCE then
						logDebug(LOG_LEVELS.INFO, "Direct movement complete, close enough to seat. Transitioning to ATTEMPTING_SIT.")
						setState(States.ATTEMPTING_SIT)
					else
						logDebug(LOG_LEVELS.WARN, "Direct movement complete but still not close enough to seat. Distance:", newDistance)
						-- Stay in MOVING_TO_SEAT, will try again next tick
					end
				else
					logDebug(LOG_LEVELS.WARN, "Seat became invalid during direct movement. Re-finding.")
					targetSeat = nil
					setState(States.FINDING_SEAT)
				end
			end
		end
	elseif currentState == States.ATTEMPTING_SIT then
		local success = attemptToSit()
		if success then
			logDebug(LOG_LEVELS.INFO, "Successfully sat in seat. Transitioning to DRIVING_VEHICLE.")
			setState(States.DRIVING_VEHICLE)
		else
			if targetSeat and targetSeat.Parent then
				logDebug(LOG_LEVELS.WARN, "Failed to sit in seat. Retrying ATTEMPTING_SIT.")
				-- Stay in ATTEMPTING_SIT, will try again next tick
			else
				logDebug(LOG_LEVELS.WARN, "Seat became invalid during sit attempt. Re-finding.")
				targetSeat = nil
				setState(States.FINDING_SEAT)
			end
		end
	elseif currentState == States.DRIVING_VEHICLE then
		local shouldExit = driveBike()
		if shouldExit then
			logDebug(LOG_LEVELS.INFO, "Bike driving complete or needs exit. Transitioning to EXITING_VEHICLE.")
			setState(States.EXITING_VEHICLE)
		end
	elseif currentState == States.EXITING_VEHICLE then
		local success = exitVehicle()
		if success then
			logDebug(LOG_LEVELS.INFO, "Successfully exited vehicle. Transitioning to COMPLETED.") -- Go straight to completed after exiting
			setState(States.COMPLETED)
		else
			logDebug(LOG_LEVELS.WARN, "Failed to exit vehicle. Retrying EXITING_VEHICLE.")
			-- Stay in EXITING_VEHICLE, will try again next tick
		end
	elseif currentState == States.COMPLETED then
		logDebug(LOG_LEVELS.INFO, "NPC has completed all waypoints. Idling.")
		task.wait(1) -- Idle for a bit
		-- Stay in COMPLETED state
	end
	return false -- Continue loop
end
-- Main loop
local function mainLoop()
	while true do
		local shouldStop = updateStateMachine()
		if shouldStop then break end
		task.wait(0.1) -- Small wait to prevent tight loops
	end
	logDebug(LOG_LEVELS.INFO, "NPC Main loop stopped.")
end
-- Start the main loop
task.spawn(mainLoop)
-- Cleanup on script destruction
script.Destroying:Connect(function()
	clearWaypointVisualizations()
	logDebug(LOG_LEVELS.INFO, "NPC script destroyed, cleaned up visualizations.")
end)
logDebug(LOG_LEVELS.INFO, "NPC script initialized successfully.")

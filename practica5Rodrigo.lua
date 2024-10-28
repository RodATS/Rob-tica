-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random())) *
            math.cos(2 * math.pi * math.random()) + mean
end


function createRandomBumpyFloor()
    print ("Generating new random bumpy floor.")
    sim.setThreadAutomaticSwitch(false)

    -- Remove existing bumpy floor if there already is one
    if (heightField ~= nil) then
        sim.setObjectPosition(heightField, heightField, {0.05, 0, 0})
        return
    end
    --  Create random bumpy floor for robot to drive on
    floorSize = 5
    --heightFieldResolution = 0.3
    --heightFieldNoise = 0.00000005
    heightFieldResolution = 0.1
    heightFieldNoise = 0.0000008
    cellsPerSide = floorSize / heightFieldResolution
    cellHeights = {}
    for i=1,cellsPerSide*cellsPerSide,1 do
        table.insert(cellHeights, gaussian(0, heightFieldNoise))
    end
    heightField=sim.createHeightfieldShape(0, 0, cellsPerSide, cellsPerSide, floorSize, cellHeights)
    -- Make the floor invisible
    sim.setObjectInt32Param(heightField,10,0)
    sim.setThreadAutomaticSwitch(true)
end


function get_walls()
    -- Disable error reporting
    local savedState=sim.getInt32Param(sim.intparam_error_report_mode)
    sim.setInt32Param(sim.intparam_error_report_mode,0)
    local N = 1
    while true do
        local handle = sim.getObjectHandle("Wall"..tostring(N))
        if handle <= 0 then
            break
        end

        -- Read position and shape of wall
        -- Assume here that it is thin and oriented either along the x axis or y axis

        -- We can now get the propertries of these walls, e.g....
        local pos = sim.getObjectPosition(handle, -1)
        local res,minx = sim.getObjectFloatParameter(handle,15)
        local res,maxx = sim.getObjectFloatParameter(handle,18)
        local res,miny = sim.getObjectFloatParameter(handle,16)
        local res,maxy = sim.getObjectFloatParameter(handle,19)

        --print("Position of Wall " .. tostring(N) .. ": " .. tostring(pos[1]) .. "," .. tostring(pos[2]) .. "," .. tostring(pos[3]))
        --print("minmax", minx, maxx, miny, maxy)

        local Ax, Ay, Bx, By
        if (maxx - minx > maxy - miny) then
            print("Wall " ..tostring(N).. " along x axis")
            Ax = pos[1] + minx
            Ay = pos[2]
            Bx = pos[1] + maxx
            By = pos[2]
        else
            print("Wall " ..tostring(N).. " along y axis")
            Ax = pos[1]
            Ay = pos[2] + miny
            Bx = pos[1]
            By = pos[2] + maxy
        end
        print (Ax, Ay, Bx, By)

        walls[N] = {Ax, Ay, Bx, By}
        N = N + 1
    end
    -- enable error reporting
    sim.setInt32Param(sim.intparam_error_report_mode,savedState)

    return N - 1
end


-- This function is executed exactly once when the scene is initialised
function sysCall_init()
    tt = sim.getSimulationTime()
    print("Init hello", tt)

    robotBase=sim.getObjectHandle(sim.handle_self) -- robot handle
    leftMotor=sim.getObjectHandle("leftMotor") -- Handle of the left motor
    rightMotor=sim.getObjectHandle("rightMotor") -- Handle of the right motor
    turretMotor=sim.getObjectHandle("turretMotor") -- Handle of the turret motor
    turretSensor=sim.getObjectHandle("turretSensor")
 
    -- Create bumpy floor for robot to drive on
    createRandomBumpyFloor()

    -- Usual rotation rate for wheels (radians per second)
    speedBase = 5
    speedBaseL = 0
    speedBaseR = 0

    -- Which step are we in?
    -- 0 is a dummy value which is immediately completed
    stepCounter = 0
    stepCompletedFlag = false

    -- Sequential state machine (executed for each waypoint)
    stepList = {}
    stepList[1] = {"set_waypoint"}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"}
    stepList[4] = {"forward"}
    stepList[5] = {"stop"}
    stepList[6] = {"measure_actualization"}
    stepList[7] = {"repeat"}

    -- Waypoints
    N_WAYPOINTS = 26
    currentWaypoint = 1 --indice del way point
    waypoints = {}
    waypoints[1] = {0.5,0}
    waypoints[2] = {1,0}
    waypoints[3] = {1,0.5}
    waypoints[4] = {1,1}
    waypoints[5] = {1,1.5}
    waypoints[6] = {1,2}
    waypoints[7] = {0.5,2}
    waypoints[8] = {0,2}
    waypoints[9] = {-0.5,2}
    waypoints[10] = {-1,2}
    waypoints[11] = {-1,1.5}
    waypoints[12] = {-1,1}
    waypoints[13] = {-1.5,1}
    waypoints[14] = {-2,1}
    waypoints[15] = {-2,0.5}
    waypoints[16] = {-2,0}
    waypoints[17] = {-2,-0.5}
    waypoints[18] = {-1.5,-1}
    waypoints[19] = {-1,-1.5}
    waypoints[20] = {-0.5,-1.5}
    waypoints[21] = {0,-1.5}
    waypoints[22] = {0.5,-1.5}
    waypoints[23] = {1,-1.5} 
    waypoints[24] = {1,-1}
    waypoints[25] = {0.5,-0.5}
    waypoints[26] = {0,0}

    --Determina la diferencia entre angulos consecutivos cuando la torreta gira
    turretAngleDeltaRad = math.rad(10)
    turretAngleTarget = -(math.pi - 0.01)
    sim.setJointTargetPosition(turretMotor, turretAngleTarget)

    -- Record a series of measurements to update particles together (only need to resample once)
    distanceMeasurements = {}
    turretAngleRads = {}

    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    numberOfParticles = 100
    for i=1, numberOfParticles do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        weightArray[i] = 1.0/numberOfParticles
        dummyArray[i] = sim.createDummy(0.05) 
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
    end

    -- Variables que indicar?n al robot cuanto debe girar o avanzar
    giroRotacion = 0.0 
    distanciaAvanzar = 0.0
    
    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls() -- Modifis "walls" and returns number of walls
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates

    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0

     -- Motor angles in radians per unit (to calibrate)
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05

    -- Zero mean Gaussian noise variance in meter/radians (to calibrate)
    -- Determined for a one meter distance
    linearectaVarianza = 0.05
    linearectaAnguloVarianza = 0.05
    thetaVarianza = 0.007
end

function sysCall_sensing()
    
end

--manipulacion de particulas funciones - MOvimientos

function actualizarDummys()
    for i=1, numberOfParticles do
       sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
       sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
    end
end


function actualizacionLineaRecta(distanciaParaAvanzar)
    if (distanciaParaAvanzar == 0) then
        return
    end

    for i=1, numberOfParticles do
        xArray[i] = xArray[i] + (distanciaParaAvanzar + gaussian(0, linearectaVarianza* distanciaParaAvanzar)) * math.cos(thetaArray[i])
        yArray[i] = yArray[i] + (distanciaParaAvanzar + gaussian(0, linearectaVarianza* distanciaParaAvanzar)) * math.sin(thetaArray[i])
        thetaArray[i] = thetaArray[i] + gaussian(0, linearectaAnguloVarianza * distanciaParaAvanzar)
    end

    actualizarDummys()

    print("Actualizar particulas - linea recta")
end

function actualizacionRotacion(giroRadianes)
    if (giroRadianes == 0) then
        return
    end

    for i=1, numberOfParticles do
        thetaArray[i] = thetaArray[i] + giroRadianes + gaussian (0,math.rad(thetaVarianza))
    end

    actualizarDummys()

    print("Actualizar particulas - rotacion")
end

-------------------------------------------

function euclideanDistance(x1, y1, x2, y2)
    return math.sqrt((x1-x2)^2 + (y1-y2)^2)
end


-- Retorna Verdadero si el punto de interseccion est? entre (Ax, Ay) y (Bx, By).
function isPointOnLineBetweenTwoPoints(x, y, Ax, Ay, Bx, By)
    local distanceMargin = 0.01 -- 
    return math.abs(euclideanDistance(Ax, Ay, x, y) + euclideanDistance(x, y, Bx, By) - euclideanDistance(Ax, Ay, Bx, By)) < distanceMargin
end


-- (x, y, theta) de la particula , z dinstancia del sensor
function calculateLikelihood(x, y, theta, z)
    -- distancia esperada a la pared mas cercana
    local m = math.huge
    --ver cada pared por sus coordenadas
    for _, wall in ipairs(walls) do
        Ax = wall[1]
        Ay = wall[2]
        Bx = wall[3]
        By = wall[4]
        
        --distancia a la interseccion
        local distanceToWall = ((By - Ay)*(Ax - x) - (Bx - Ax)*(Ay - y)) / ((By - Ay)*math.cos(theta) - (Bx - Ax)*math.sin(theta))

        --la distancia es menor a la esperada (m)
        if (distanceToWall < m and distanceToWall >= 0) then
            -- identificar donde intersecta a la pared el sensor
            local intersectX = x + distanceToWall * math.cos(theta)
            local intersectY = y + distanceToWall * math.sin(theta)
            
            --verificar que el sonar choca con la pared para actualizar m
            if (isPointOnLineBetweenTwoPoints(intersectX, intersectY, Ax, Ay, Bx, By)) then
                m = distanceToWall
            end
        end
    end
    -- probabilidad de que la medida real z coincida con la distancia esperada m
    local likelihood = math.exp(- (z - m)^2 / (2*sensorVariance))

    if (m == math.huge) then
        print("No se encontr? una pared en esa direccion: likelihood: 1")
        likelihood = 1.0
    end
    return likelihood
end


-- Suma de los elementos
function sum(array)
    local sum = 0
    for i=1, #array do
        sum = sum + array[i]
    end

    return sum
end


-- Normalizacion de las particulas
function normalizarParticulasPesos()
    local weightSum = sum(weightArray)
    for i=1, #weightArray do
        weightArray[i] = weightArray[i] / weightSum
    end
end


-- Remuestreo con rueda de ruleta
function remuestreoParticulas()
    --suma de los pesos anteriores y el actual
    local cumulativeWeightArray = {weightArray[1]}
    for i=2, numberOfParticles do
        cumulativeWeightArray[i] = cumulativeWeightArray[i-1] + weightArray[i]
    end
    local newXArray = {}
    local newYArray = {}
    local newThetaArray = {}
    for i=1, numberOfParticles do
        local r = math.random() -- Random number in range [0,1]
        for j=1, #cumulativeWeightArray do
            --se busca la particula que cumpla con la condicion
            if (r <= cumulativeWeightArray[j]) then
                --se reemplaza la paritcula
                newXArray[i] = xArray[j]
                newYArray[i] = yArray[j]
                newThetaArray[i] = thetaArray[j]
                break
            end
        end
    end
    --se reemplazan por las nuevas particulas
    xArray = newXArray
    yArray = newYArray
    thetaArray = newThetaArray
    
    --se reinician los pesos de las particulas
    for i=1, numberOfParticles do
        weightArray[i] = 1 / numberOfParticles
    end
end


-- Actualizar las particulas con la medicion del sensor
function actualizarParticulasDespuesMedicion(distanceMeasurements, anguloTorreta)
    for i=1, numberOfParticles do
        for j=1, #distanceMeasurements do
             -- Calculo del likehood con las mediciones del sensor por cada particula por cada pared detectada
            local likelihood = calculateLikelihood(xArray[i], yArray[i], thetaArray[i] + anguloTorreta[j], distanceMeasurements[j])
            weightArray[i] = weightArray[i] * likelihood
        end
    end

    normalizarParticulasPesos()

    remuestreoParticulas()

    actualizarDummys()

    print("Actualizar medicion, normalizacion, y remuestreo: listo")
    print("Se actualizaron las particulas")
end


---
----------------------------------------
function calcularCentroide()
    local sumaX = 0
    local sumaY = 0
    local sumaTheta = 0
    local N = 100  -- N?mero de part?culas

    for i = 1, N do
        sumaX = sumaX + xArray[i]*weightArray[i]
        sumaY = sumaY + yArray[i]*weightArray[i]
        sumaTheta = sumaTheta + thetaArray[i]*weightArray[i]
    end


    -- Ajustar el promedio de theta para estar en el rango de -pi a pi
    while sumaTheta > math.pi do
        sumaTheta = sumaTheta - 2 * math.pi
    end
    while sumaTheta < -math.pi do
        sumaTheta = sumaTheta + 2 * math.pi
    end

    return sumaX, sumaY, sumaTheta
end


--calculcar angulo de giro
function calcularAngulo(xActual, yActual, xObjetivo, yObjetivo)
    return math.atan2(yObjetivo - yActual, xObjetivo - xActual)
end


---------------------------------------------------


-- How far are the left and right motors from their targets? Find the maximum
function getMaxMotorAngleFromTarget(posL, posR)
    maxAngle = 0
    if (speedBaseL > 0) then
        remaining = motorAngleTargetL - posL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseL < 0) then
        remaining = posL - motorAngleTargetL
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR > 0) then
        remaining = motorAngleTargetR - posR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end
    if (speedBaseR < 0) then
        remaining = posR - motorAngleTargetR
        if (remaining > maxAngle) then
            maxAngle = remaining
        end
    end

    return maxAngle
end


function sysCall_actuation() 
    tt = sim.getSimulationTime() 

    -- Get current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)

    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false

        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]

            if (currentWaypoint == N_WAYPOINTS) then
                currentWaypoint = 1
            else
                currentWaypoint = currentWaypoint + 1
            end
        end

        print("New step:", stepCounter, newStepType)

        if (newStepType == "set_waypoint") then
        
            local waypointX = waypoints[currentWaypoint][1]
            local waypointY = waypoints[currentWaypoint][2]

            -- Calcular el angulo hacia el siguiente waypoint
            local xActual, yActual, thetaActual = calcularCentroide()
            local anguloObjetivo = calcularAngulo(xActual, yActual, waypointX, waypointY)
            giroRotacion = anguloObjetivo - thetaActual

            -- Asegurarse de que el giro sea el menor posible
            if giroRotacion > math.pi then
                giroRotacion = giroRotacion - 2 * math.pi
            elseif giroRotacion < -math.pi then
                giroRotacion = giroRotacion + 2 * math.pi
            end

            distanciaAvanzar =  euclideanDistance(xActual, yActual, waypointX, waypointY)
            print("giro", giroRotacion)
            print("avanzar", distanciaAvanzar)
            
            
        elseif (newStepType == "forward") then
            -- Forward step: set new joint targets
            motorAngleTargetL = posL + distanciaAvanzar * motorAnglePerMetre
            motorAngleTargetR = posR + distanciaAvanzar * motorAnglePerMetre
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            motorAngleTargetL = posL - giroRotacion  * motorAnglePerRadian
            motorAngleTargetR = posR + giroRotacion  * motorAnglePerRadian
        elseif (newStepType == "stop") then
            print("Stopping")
        elseif (newStepType == "measure_actualization") then
            print("Taking measurements")
        end
    end

    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "set_waypoint") then
        stepCompletedFlag = true
        
    elseif (stepType == "turn") then
    
        if (giroRotacion >= 0) then
            -- girar a la izquierda
            speedBaseL = -speedBase
            speedBaseR = speedBase
        else
            -- girar a la derecha
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end

        local motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- bajar la velocidad
        if (motorAngleFromTarget < 3) then
            local speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        -- si llego a la meta
        if (motorAngleFromTarget == 0.0) then
            stepCompletedFlag = true

            -- actualziar particulas
            actualizacionRotacion(giroRotacion)
            giroRotacion = 0.0
        end
        
    elseif (stepType == "forward") then
        speedBaseL = speedBase
        speedBaseR = speedBase

        local motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Bajar velocidad
        if (motorAngleFromTarget < 3) then
            local speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        -- llego a la meta
        if (motorAngleFromTarget == 0.0) then
            stepCompletedFlag = true

            -- actualizar particulas
            actualizacionLineaRecta(distanciaAvanzar)
            distanciaAvanzar = 0.0
        end
    elseif (stepType == "stop") then
        speedBaseL = 0
        speedBaseR = 0

        -- Check to see if the robot is stationary to within a small threshold
        local linearVelocity, angularVelocity = sim.getVelocity(robotBase)
        local vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        local vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    elseif (stepType == "measure_actualization") then
        
        speedBaseL = 0
        speedBaseR = 0

        --Verificar que la pos de la torreta tenga un error minimo al objetivo
        if math.abs(sim.getJointPosition(turretMotor) - turretAngleTarget) < 0.01 then
            --si detecta algo la torreta
            local result, cleanDistance = sim.readProximitySensor(turretSensor)
            if result > 0 then
                distanceMeasurements[#distanceMeasurements+1] = cleanDistance + gaussian(0.0, sensorVariance)
                turretAngleRads[#turretAngleRads+1] = turretAngleTarget
            end

            --actualizar nuevo angulo objetivo de la torreta para la sig medicion
            turretAngleTarget = turretAngleTarget + turretAngleDeltaRad
            
        --acabo de medir todo, la torreta ya escaneo todo    
        elseif turretAngleTarget + turretAngleDeltaRad >= (math.pi - 0.01) then
            
            -- Aplicacion monte carlo actualizar particulas
            actualizarParticulasDespuesMedicion(distanceMeasurements, turretAngleRads)
            --reiniciar las mediciones del sensor
            distanceMeasurements = {}
            turretAngleRads = {}

            -- Reset sensor
            turretAngleTarget = -(math.pi - 0.01)
            sim.setJointTargetPosition(turretMotor, turretAngleTarget)

            stepCompletedFlag = true
        else
            -- Rotar sensor torreta
            sim.setJointTargetPosition(turretMotor, turretAngleTarget)
        end
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)
end


function sleep(seconds)
    local t0 = os.clock()
    while os.clock() - t0 <= seconds do end
end


function sysCall_cleanup()
    --simUI.destroy(ui)
end

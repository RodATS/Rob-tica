-- Generate a sample from a Gaussian distribution
function gaussian (mean, variance)
    return  math.sqrt(-2 * variance * math.log(math.random() + 0.00001)) *
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
    stepList = {}

    N_WAYPOINTS = 26
    currentWaypoint = 0
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

    stepList[1] = {"set_waypoint"}
    stepList[2] = {"turn"}
    stepList[3] = {"stop"}
    stepList[4] = {"forward"}
    stepList[5] = {"stop"}
    stepList[6] = {"repeat"}
 
 
    -- Data structure for walls
    walls = {}
    -- Fill it by parsing the scene in the GUI
    N_WALLS = get_walls()
    -- walls now is an array of arrays with the {Ax, Ay, Bx, By} wall coordinates
    
    -- Create and initialise arrays for particles, and display them with dummies
    xArray = {}
    yArray = {}
    thetaArray = {}
    weightArray = {}
    dummyArray = {}
    N = 100
    for i=1, N do
        xArray[i] = 0
        yArray[i] = 0
        thetaArray[i] = 0
        weightArray[i] = 1/N
        dummyArray[i] = sim.createDummy(0.05)
        sim.setObjectPosition(dummyArray[i], -1, {0,0,0})
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,0})
    end
    
    -- Target positions for joints
    motorAngleTargetL = 0.0
    motorAngleTargetR = 0.0

     -- To calibrate
    motorAnglePerMetre = 24.8
    motorAnglePerRadian = 3.05
    
    sensorStandardDeviation = 0.1
    sensorVariance = sensorStandardDeviation^2
    noisyDistance = 0
end

function sysCall_sensing()
    
end

--Variables Adicionales
    centroide_robot = {x=0,y=0,theta=0}
    waypoint = {x = 0, y = 0}
    movimiento = {dist = 0, ang = 0}
    tabla_centroides = {}
    index_waypoint = 1
    proxim_muro = 1
    
--Funciones Adicionales
function getMaxMotorAngleFromTarget(posL, posR)

    -- How far are the left and right motors from their targets? Find the maximum
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

function actualizarParticulasMovimientoRecto(D,varianza_dist,varianza_giro) --Cambiar valores al moverse
    N = 100
    for i=1, N do
        e = gaussian(0,varianza_dist)
        f_ = gaussian(0,varianza_giro)
        xArray[i] = xArray[i] + (D+e)*math.cos(thetaArray[i])
        yArray[i] = yArray[i] + (D+e)*math.sin(thetaArray[i])
        thetaArray[i] = thetaArray[i] + f_ 
        weightArray[i] = 1/N
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
    end
end

function actualizarParticulasGiro(varianza,alpha) -- Cambiar valores al girar
    N = 100
    for i=1, N do
        g = gaussian(0,varianza)
        thetaArray[i] = thetaArray[i] + alpha + g
        weightArray[i] = 1/N
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
        
    end
end

function centroide(N,punto) -- Hallar el centroide
    x_ = 0
    y_ = 0
    theta_ = 0
    for i=1, N do
        x_  = x_ + xArray[i]*weightArray[i]
        y_  = y_ + yArray[i]*weightArray[i]
        theta_  = theta_ + thetaArray[i]*weightArray[i]
    end
    
    punto.x = x_
    punto.y = y_
    punto.theta = theta_
    
end

function calcularDistancia(A,x,y) -- X,Y son los valores del punto objetivo
    dx = x - A.x
    dy = y - A.y
    return math.sqrt(dx * dx + dy * dy)
end

function calcularAngulo(A,x,y)

    dx = x - A.x  
    dy = y - A.y
    
    theta_AB = math.atan2(dy, dx)
    
    delta_theta = theta_AB - A.theta
    
    delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi
    
    return delta_theta
end

function agregarPunto(tabla, x, y, theta)
    local nuevoPunto = {x = x, y = y, theta = theta}
    table.insert(tabla, nuevoPunto)
end

--funcion para M
function calcularM(i, x, y, theta)
    num = (walls[i][4] - walls[i][2])*(walls[i][1] - x)
    num2 = (walls[i][3] - walls[i][1])*(walls[i][2] - y)
    denom = (walls[i][4] - walls[i][2])*math.cos(theta)
    denom2 = (walls[i][3] - walls[i][1])*math.sin(theta)
    m = (num-num2)/(denom-denom2)
    return m
end

-- Funcion para encontrar el muro mas cercano
function hallarMuro(centroidX, centroidY,theta, noisyDistance)
    indice_cercano = -1
    minDiferencia = math.huge

    for i = 1, N_WALLS do
        
        M = calcularM(i, centroidX, centroidY, theta)
        diff = math.abs(M - noisyDistance)

        if diff < minDiferencia then
            minDiferencia = diff
            indice_cercano = i
        end
    end

    return indice_cercano
end

--Funcion likelihood
function likelihood(x,y,theta,z, proxim_muro)
    m = calcularM(proxim_muro,x,y,theta)
    p = math.exp((-((z-m)*(z-m))) / (2*sensorStandardDeviation^2))
    --es lo mismo usar sensorVariance en el denominador
    return p
end

-- Funcion actualizae wi
function actualizarPesos(z,proxim_muro)
    N = 100
    sum_w = 0
    for i=1, N do
        weightArray[i] = weightArray[i] * likelihood(xArray[i],yArray[i],thetaArray[i],z,proxim_muro)
        sum_w = sum_w + weightArray[i] 
    end
    
    -- normalizar los pesos
    if sum_w > 0 then  
        for i = 1, N do
            weightArray[i] = weightArray[i] / sum_w
        end
    else
        for i = 1, N do
            weightArray[i] = 1 / N 
        end
    end
    
end

--Funcion de remuestreo

function remuestrear()
    N = 100
    nuevaParticulaArray = {}
    
    pesosAcumulados = {}
    acumulado = 0
    
    for i = 1, N do
        acumulado = acumulado + weightArray[i]
        pesosAcumulados[i] = acumulado
    end
    
    for i = 1, N do
        local r = math.random() * acumulado
        
        for j = 1, N do
            if r <= pesosAcumulados[j] then
                nuevaParticulaArray[i] = {
                    x = xArray[j],
                    y = yArray[j],
                    theta = thetaArray[j]
                }
                break
            end
        end
    end
    
    for i = 1, N do
        xArray[i] = nuevaParticulaArray[i].x
        yArray[i] = nuevaParticulaArray[i].y
        thetaArray[i] = nuevaParticulaArray[i].theta
        weightArray[i] = 1/N
        sim.setObjectOrientation(dummyArray[i], -1, {0,0,thetaArray[i]})
        sim.setObjectPosition(dummyArray[i], -1, {xArray[i],yArray[i],0})
    end
    print("Remuestreado")
end --

function sysCall_actuation() 
    tt = sim.getSimulationTime()
    -- print("actuation hello", tt)
    
    -- Get and plot current angles of motor joints
    posL = sim.getJointPosition(leftMotor)
    posR = sim.getJointPosition(rightMotor)
 

    result,cleanDistance=sim.readProximitySensor(turretSensor)
    
    if (result>0) then
        noisyDistance= cleanDistance + gaussian(0.0, sensorVariance)
        --print ("Depth sensor reading ", noisyDistance)
    end
    
    -- Start new step?
    if (stepCompletedFlag == true or stepCounter == 0) then
        stepCounter = stepCounter + 1
        stepCompletedFlag = false

        newStepType = stepList[stepCounter][1]

        if (newStepType == "repeat") then
            -- Loop back to the first step
            stepCounter = 1
            newStepType = stepList[stepCounter][1]
        end
        
        if (newStepType == "set_waypoint") then
            waypoint.x = waypoints[index_waypoint][1]
            waypoint.y = waypoints[index_waypoint][2]
            print(string.format("Wx: %f Wy: %f",waypoint.x,waypoint.y))
            movimiento.dist = calcularDistancia(centroide_robot,waypoint.x,waypoint.y)
            movimiento.ang = calcularAngulo(centroide_robot,waypoint.x,waypoint.y)
            print(string.format("Distancia al waypoint: %f",movimiento.dist))
            print(string.format("Angulo de giro al waypoint: %f",movimiento.ang))
            
            if (index_waypoint < 26) then
                index_waypoint = index_waypoint + 1
            else
                index_waypoint = 1
            end
        end

        print("New step:", stepCounter, newStepType)
 
        if (newStepType == "forward") then
            -- Forward step: set new joint targets
            --newStepAmount = stepList[stepCounter][2]
            newStepAmount = movimiento.dist
            motorAngleTargetL = posL + newStepAmount * motorAnglePerMetre
            motorAngleTargetR = posR + newStepAmount * motorAnglePerMetre
        elseif (newStepType == "turn") then
            -- Turn step: set new targets
            newStepAmount = movimiento.ang
            motorAngleTargetL = posL - newStepAmount * motorAnglePerRadian
            motorAngleTargetR = posR + newStepAmount * motorAnglePerRadian
        elseif (newStepType == "stop") then
            print ("Stopping!")
            
        end
        
        
    end


    -- Handle current ongoing step
    stepType = stepList[stepCounter][1]

    if (stepType == "turn") then
        if (movimiento.ang > 0) then
            speedBaseL = -speedBase
            speedBaseR = speedBase
        elseif (movimiento.ang < 0) then
            speedBaseL = speedBase
            speedBaseR = -speedBase
        end
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            actualizarParticulasGiro(math.rad(0.5),movimiento.ang)
            centroide(100,centroide_robot)
            --pesos--
            proxim_muro = hallarMuro(centroide_robot.x,centroide_robot.y,centroide_robot.theta,noisyDistance)
            print(string.format(proxim_muro))
            actualizarPesos(noisyDistance,proxim_muro)
            --Remuestrar--
            remuestrear()
            --------------
            centroide(100,centroide_robot)
            print(string.format("CX %f CY %f CTheta %f",centroide_robot.x,centroide_robot.y,centroide_robot.theta))
            agregarPunto(tabla_centroides,centroide_robot.x,centroide_robot.y,centroide_robot.theta)
            stepCompletedFlag = true
        end
    elseif (stepType == "forward") then
        speedBaseL = speedBase
        speedBaseR = speedBase
        motorAngleFromTarget = getMaxMotorAngleFromTarget(posL, posR)
        -- Slow down when close
        if (motorAngleFromTarget < 3) then
            speedScaling = 0.2 + 0.8 * motorAngleFromTarget / 3
            speedBaseL = speedBaseL * speedScaling
            speedBaseR = speedBaseR * speedScaling
        end
        if (motorAngleFromTarget == 0) then
            actualizarParticulasMovimientoRecto(movimiento.dist,sensorVariance,math.rad(0.5))
            centroide(100,centroide_robot)
            --pesos--
            proxim_muro = hallarMuro(centroide_robot.x,centroide_robot.y,centroide_robot.theta,noisyDistance)
            print(string.format(proxim_muro))
            actualizarPesos(noisyDistance,proxim_muro)
            --Remuestrar--
            remuestrear()
            --------------
            centroide(100,centroide_robot)
            print(string.format("CX %f CY %f CTheta %f",centroide_robot.x,centroide_robot.y,centroide_robot.theta))
            agregarPunto(tabla_centroides,centroide_robot.x,centroide_robot.y,centroide_robot.theta)
            stepCompletedFlag = true
        end
    elseif (stepType == "stop") then
        speedBaseL = 0
        speedBaseR = 0
        
        --proxim_muro = hallarMuro(centroide_robot.x,centroide_robot.y,centroide_robot.theta,noisyDistance)
        --print(string.format(proxim_muro))
        
        -- Check to see if the robot is stationary to within a small threshold
        linearVelocity,angularVelocity=sim.getVelocity(robotBase)
        vLin = math.sqrt(linearVelocity[1]^2 + linearVelocity[2]^2 + linearVelocity[3]^2)
        vAng = math.sqrt(angularVelocity[1]^2 + angularVelocity[2]^2 + angularVelocity[3]^2)
        --print ("stop", linearVelocity, vLin, vAng)
    
        if (vLin < 0.001 and vAng < 0.01) then
            stepCompletedFlag = true
        end
    elseif (stepType == "set_waypoint") then
        stepCompletedFlag = true
    end

    -- Set the motor velocities for the current step
    sim.setJointTargetVelocity(leftMotor,speedBaseL)
    sim.setJointTargetVelocity(rightMotor,speedBaseR)

end

function sysCall_cleanup()
    --simUI.destroy(ui)
end 

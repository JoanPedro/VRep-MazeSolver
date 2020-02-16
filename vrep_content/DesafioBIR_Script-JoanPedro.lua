function  callRotateToRight(theSalt, angle)
    stripRotate = 0
    theSalt = 0
    if(e1[3] < 0) then
        e1[3] = -e1[3]
    end
    while(theSalt < (angle*0.985)) do
        
        rotateToRight(theSalt, angle)
        if(e1[3] > 0) then
            theSalt = e1[3] - e[3]
        else
            theSalt = e1[3] + e[3]
        end
    --resultAngle = e[3] - angle
    --sim.addStatusbarMessage(e1[3])
    --sim.addStatusbarMessage(theSalt)
    --sim.addStatusbarMessage(theSalt)
    --sim.addStatusbarMessage(angle)
    --sim.addStatusbarMessage(e[3])
    
    end
    
    stripRotate = 1
    
end

function rotateToRight(theSalt, angle)
    
    if(theSalt < angle*0.95) then
        vLeft = 180*math.pi/180
        vRight = -180*math.pi/180
    else
        vLeft = 05*math.pi/180
        vRight = -05*math.pi/180
    end
    local m=sim.getObjectMatrix(bodyElements,-1)
    local m2=sim.buildMatrix({0,0,0},{0,90*math.pi/180,90*math.pi/180})
    m=sim.multiplyMatrices(m,m2)
    e=sim.getEulerAnglesFromMatrix(m)
    
    sim.setJointTargetVelocity(motorHandles[1],vLeft)
    sim.setJointTargetVelocity(motorHandles[2],-vRight)
    sim.setJointTargetVelocity(motorHandles[3],-vRight)
    sim.setJointTargetVelocity(motorHandles[4],vLeft)
    

    
end

function  callRotateToLeft(theSalt, angle)
    stripRotate = 0

    while(theSalt < (angle*0.985)) do
            
--    if(e[1] > 0) then
--        e[1] = e[1]
--    else
--        e[1] = -e[1]
--    end
        rotateToLeft(theSalt, angle)
        if(e1[3] > 0) then
            theSalt = e1[3] + e[3]
        else
            theSalt = e[3] - e1[3]
        end
    --resultAngle = e[3] - angle
    --sim.addStatusbarMessage(e1[3])
    --sim.addStatusbarMessage(theSalt)
    --sim.addStatusbarMessage(e[3])
    end
    
    stripRotate = 1
end

function rotateToLeft(theSalt, angle)  

    if(theSalt < angle*0.95) then
        vLeft = -180*math.pi/180
        vRight = 180*math.pi/180
    else
        vLeft = -05*math.pi/180
        vRight = 05*math.pi/180
    end

    local m=sim.getObjectMatrix(bodyElements,-1)
    local m2=sim.buildMatrix({0,0,0},{0,90*math.pi/180,90*math.pi/180})
    m=sim.multiplyMatrices(m,m2)
    e=sim.getEulerAnglesFromMatrix(m)
    
    sim.setJointTargetVelocity(motorHandles[1],vLeft)
    sim.setJointTargetVelocity(motorHandles[2],-vRight)
    sim.setJointTargetVelocity(motorHandles[3],-vRight)
    sim.setJointTargetVelocity(motorHandles[4],vLeft)
    
end

function callGoForward(vLeft, vRight)

    goForward(vLeft, vRight)
end

function goForward(vLeft, vRight)
    sim.setJointTargetVelocity(motorHandles[1],vLeft)
    sim.setJointTargetVelocity(motorHandles[2],-vRight)
    sim.setJointTargetVelocity(motorHandles[3],-vRight)
    sim.setJointTargetVelocity(motorHandles[4],vLeft)
end

function sysCall_threadmain()

    -- This part run once
    
    usensors={-1,-1,-1, -1, -1}
    for i=1,5,1 do
        usensors[i]=sim.getObjectHandle('Robotnik_Proximity'..i)
    end
    
    uvision = {-1}
    uvision = sim.getObjectHandle('Vision_sensor')
    
    motorHandles={-1,-1,-1,-1}
    barHandles={-1,-1,-1,-1}
    
    bodyElements=sim.getObjectHandle('Summit_XL_visible')
    motorHandles[1]=sim.getObjectHandle('joint_front_left_wheel')
    motorHandles[2]=sim.getObjectHandle('joint_front_right_wheel')
    motorHandles[3]=sim.getObjectHandle('joint_back_right_wheel')
    motorHandles[4]=sim.getObjectHandle('joint_back_left_wheel')
       
    noDetectionDist = 2
    maxDetectionDist = 0.4
    detect = {0,0,0,0,0,0}
    braitenbergL={-0.4,-0.8,-1.2, -1.6}
    --braitenbergL={1,2,-2,-1}
    braitenbergR={-1.6, -1.2,-0.8,-0.4}
    --braitengergF={0.485,0.5}
    braitengergF={1,1}
    v0= 4
    
    proxSensDist = {noDetectionDist,noDetectionDist, noDetectionDist, noDetectionDist, noDetectionDist}
    -- Prepare 2 floating views with the camera views:
    frontCam=sim.getObjectHandle('Robotnik_frontCamera')
    
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    
    sim.adjustView(floorView,frontCam,64)
    sim.adjustView(frontView,frontCam,64)

    forwarding = {-1,-1, -1, -1}

    theSalt = 0
    theSalt2 = 0
    angle = 90*math.pi/180
    stripRotate = 0
    endPoit = 0
    -- End of once run. Its like a "Setup" on arduino IDE. 
    
    -- Start of main Loop. 
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        local m=sim.getObjectMatrix(bodyElements,-1)
        local m2=sim.buildMatrix({0,0,0},{0,90*math.pi/180,90*math.pi/180})
        m=sim.multiplyMatrices(m,m2)
        e1=sim.getEulerAnglesFromMatrix(m)

        
        res_2, t0, t1 = sim.readVisionSensor(uvision)
        --sim.addStatusbarMessage(res_2)
        st = sim.getSimulationTime()
        vLeft= v0
        vRight= v0
        s=sim.getObjectSizeFactor(bodyElements)
        noDetectionDistance=0.05*s
        for i=1,5,1 do
            res,dist=sim.readProximitySensor(usensors[i])
            if (res>0) and (dist<noDetectionDist) then
                proxSensDist[i] = dist
                if (dist<maxDetectionDist) then
                    dist=maxDetectionDist
                end
                forwarding[i] = 1
                detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
            else
                forwarding[i] = 0
                detect[i]=0
            end
        end
        
        -- Endpoint
        if(forwarding[5] == 1) then
            vLeft = 0
            vRight = 0
            callGoForward(vLeft,vRight)
        else
            if(forwarding[4] == 1 and forwarding[1] == 0 ) then
                callGoForward(vLeft, vRight)
            else
                if(forwarding[1] == 0 and (forwarding[2] + forwarding[3]) == 1*2 or (forwarding[2] + forwarding[3]) == 0 ) then
                    callGoForward(vLeft, vRight)
                else
                    if(forwarding[3] == 0) then
                        callRotateToLeft(theSalt, angle)
                    else
                        if(forwarding[1] == 0 ) then
                            callGoForward(vLeft, vRight)
                        else
                            if(forwarding[1] == 1) then
                                callRotateToRight(theSalt, angle)
                            end
                        end
                    end
                end
            end
        end
    end
end


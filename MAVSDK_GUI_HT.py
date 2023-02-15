import asyncio
from tkinter import *
from turtle import home
from async_tkinter_loop import async_handler, async_mainloop
from mavsdk import *
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityBodyYawspeed)
import time
import webbrowser
import math

drone = System()
lastPacketTime=time.time()-10
t = 0
currentAlt = 0
def hyperLink(url):
    webbrowser.open_new(url)

async def setup_simulation():
    """
    General configurations, setups, and connections are done here.
    :return:
    """
    await drone.connect(system_address="udp://:"+portIn.get())

    printPxh("Waiting for drone to connect...")
    global state
    global lastPacketTime
    global health

    async for state in drone.core.connection_state():
        lastPacketTime=time.time()
        if state.is_connected:
            printPxh(f"-- Connected to drone!")
            break

    asyncio.ensure_future(checkTelem())
    asyncio.ensure_future(print_health())
    asyncio.ensure_future(print_altitude())
    asyncio.ensure_future(print_position_ned())
    asyncio.ensure_future(print_flightmode())
    asyncio.ensure_future(checkarm())
    asyncio.ensure_future(print_heading())
    asyncio.ensure_future(print_landed())

    printPxh("Waiting for drone to have a global position estimate...")
    
    while True:
        await print_health()
        if health.is_global_position_ok and health.is_home_position_ok:
            printPxh("-- Global position estimate OK")
            break

async def setup_realpx4():
    """
    General configurations, setups, and connections are done here.
    :return:
    """

    await drone.connect(system_address="serial:///dev/ttyUSB0")

    printPxh("Waiting for drone to connect...")
    global state
    global lastPacketTime
    global health

    async for state in drone.core.connection_state():
        lastPacketTime=time.time()
        if state.is_connected:
            printPxh(f"-- Connected to drone!")
            break

    asyncio.ensure_future(checkTelem())
    asyncio.ensure_future(print_health())
    asyncio.ensure_future(print_altitude())
    asyncio.ensure_future(print_position_ned())
    asyncio.ensure_future(print_flightmode())
    asyncio.ensure_future(checkarm())
    asyncio.ensure_future(print_heading())
    asyncio.ensure_future(print_landed())

    printPxh("Waiting for drone to have a global position estimate...")
    
    while True:
        await print_health()
        if health.is_global_position_ok and health.is_home_position_ok:
            printPxh("-- Global position estimate OK")
            break

async def checkTelem():
    global lastPacketTime 
    while True:
        #printPxh(str(time.time()))
        #printPxh(str(time.time() - lastPacketTime))
        if (time.time() - lastPacketTime) > 1 :
            linkTextObj.config(fg="red")
        else:
            linkTextObj.config(fg="green")
        await asyncio.sleep(3) 

async def checkarm():
    async for checkarm in drone.telemetry.armed():
        if checkarm:
            checkarmText.delete(1.0,"end")
            checkarmText.insert(1.0,"Armed")
        else:
            checkarmText.delete(1.0,"end")
            checkarmText.insert(1.0,"Disarmed")

async def increase_t():
    global t
    while(1):
        t = t + 0.1
        await asyncio.sleep(1)
        printPxh(str(t))

async def current_alt():
    global currentAlt
    async for position in drone.telemetry.position():
        currentAlt = round(position.relative_altitude_m,1)

async def shutdown():
    printPxh("Shutting Down the Drone")
    await drone.action.shutdown()

    
async def testArm():
    printPxh("-- Arming")
    await drone.action.arm()           
    await asyncio.sleep(5)
    printPxh("-- DisArming")
    await drone.action.disarm()
    
async def arm():
    printPxh("-- Initializing")
    printPxh("-- Arming")
    await drone.action.arm()

async def disarm():
    printPxh("-- Disarming")
    await drone.action.disarm()

async def change_hold_mode():
    await drone.action.hold()

async def takeoff(alt=10):
    printPxh("-- Taking off")
    await drone.action.set_takeoff_altitude(float(altIn.get()))
    await drone.action.takeoff()

async def land():
    printPxh("-- Landing")
    altIn.delete(0,END)
    altIn.insert(0,0)
    await drone.action.land()

async def offboard():
    
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    await drone.offboard.set_position_ned(PositionNedYaw(float(NorthIn.get()), float(EastIn.get()), -float(DownIn.get()), float(YawIn.get())))

async def do_a_circle():
    global position
    global t
    global nedposition
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    count = asyncio.ensure_future(increase_t())

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    r = float(radiusIn.get())
    alt = (round(position.relative_altitude_m,1))
    northpos = nedposition.position.north_m
    eastpos = nedposition.position.east_m
    while(t<20):
        await drone.offboard.set_position_ned(PositionNedYaw(float(northpos + r*math.sin(t)), float(eastpos + r*math.cos(t)), -alt, 0.0))
        await asyncio.sleep(2)
    t = 0
    count.cancel()
async def clockwise():
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 10.0))
    await asyncio.sleep(1.5)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
async def count_clockwise():
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, -10.0))
    await asyncio.sleep(1.5)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
async def forward():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(1.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(float(BodyStepIn.get()))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

async def back():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-1.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(float(BodyStepIn.get()))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

async def left():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, -1.0, 0.0, 0.0))
        await asyncio.sleep(float(BodyStepIn.get()))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

async def right():
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 1.0, 0.0, 0.0))
        await asyncio.sleep(float(BodyStepIn.get()))
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

def printPxh(msg=""):
    pxhOut.insert(END, msg + '\n')
    print(msg)
    pxhOut.see("end")

async def print_landed():
    async for landed_state in drone.telemetry.landed_state():
        LandedStateText.delete(1.0,"end")
        LandedStateText.insert(1.0,str(landed_state))

async def print_health():
        defColor = portLabelObj.cget("fg")
        async for health in drone.telemetry.health():
            #printPxh(f"Health: {health}")
            if health.is_gyrometer_calibration_ok & health.is_accelerometer_calibration_ok & health.is_magnetometer_calibration_ok :
               ahrsTextObj.config(fg="green") 
               
            if health.is_local_position_ok & health.is_global_position_ok & health.is_home_position_ok :
               posTextObj.config(fg="green") 
        
            if health.is_armable:
               armTextObj.config(fg="green") 
            global lastPacketTime   
            lastPacketTime=time.time()
async def set_max_speed():
    await drone.action.set_maximum_speed(float(maxSpeedIn.get()))

async def print_flightmode():
    global flight_mode
    async for flight_mode in drone.telemetry.flight_mode():
        FlightModeText.delete(1.0,"end")
        FlightModeText.insert(1.0,str(flight_mode))

async def print_heading():
    async for heading in drone.telemetry.heading():
        HeadingText.delete(1.0,"end")
        HeadingText.insert(1.0,str(heading))

async def print_altitude():
    global position
    async for position in drone.telemetry.position():
        altText.delete(1.0,"end")
        altText.insert(1.0,str(round(position.relative_altitude_m,1))+" m")

async def print_position_ned():
    global nedposition
    async for nedposition in drone.telemetry.position_velocity_ned():
        NedPosText.delete(1.0,"end")
        NedPosText.insert(1.0,"N: "+str(round(nedposition.position.north_m,1))+"m, "+"E: "+str(round(nedposition.position.east_m,1))+"m, "+"D: "+str(round(nedposition.position.down_m,1))+"m")

root = Tk()
root.geometry("1360x700")
root.title("PX4 MAVSDK GUI HIEP & TUYEN ")

labelPortText=StringVar()
labelPortText.set("Receiving Simulation Port: ")
portLabelObj=Label(root, textvariable=labelPortText, height=1)
portLabelObj.grid(row=0,column=1,rowspan=1,columnspan=1)

defPort = StringVar(root, value='14540')
portIn = Entry(root, textvariable=defPort)
portIn.grid(row=0,column=2,rowspan=1,columnspan=1)
Button(root, text="Connect", command=async_handler(setup_simulation)).grid(row=0,column=2,rowspan=1,columnspan=2)

labelRealPx4Text=StringVar()
labelRealPx4Text.set("Connect Real PX4: ")
RealPx4LabelObj=Label(root, textvariable=labelRealPx4Text, height=1)
RealPx4LabelObj.grid(row=1,column=1,rowspan=1,columnspan=1)

Button(root, text="Connect", command=async_handler(setup_realpx4)).grid(row=1,column=1,rowspan=1, columnspan=2)

posTextStr=StringVar()
posTextStr.set("NAV")
posTextObj=Label(root, textvariable=posTextStr, height=1)
posTextObj.grid(row=2,column=1,rowspan=1,columnspan=1)
posTextObj.config(fg= "red")

ahrsTextStr=StringVar()
ahrsTextStr.set("AHRS")
ahrsTextObj=Label(root, textvariable=ahrsTextStr, height=1)
ahrsTextObj.grid(row=2,column=2,rowspan=1,columnspan=1)
ahrsTextObj.config(fg= "red")


linkTextStr=StringVar()
linkTextStr.set("LINK")
linkTextObj=Label(root, textvariable=linkTextStr, height=1)
linkTextObj.grid(row=3,column=1,rowspan=1,columnspan=1)
linkTextObj.config(fg= "red")

armTextStr=StringVar()
armTextStr.set("READY")
armTextObj=Label(root, textvariable=armTextStr, height=1)
armTextObj.grid(row=3,column=2,rowspan=1,columnspan=1)
armTextObj.config(fg= "red")

labelcheckarmText = StringVar()
labelcheckarmText.set("Armed/Disarmed: ")
checkarmLabel = Label(root, textvariable=labelcheckarmText, height=1)
checkarmLabel.grid(row=4,column=1,rowspan=1)

checkarmText = Text(root, height=1, width = 15)
checkarmText.grid(row = 4, column = 1, rowspan = 1,columnspan=2)
checkarmText.insert(END,"UNKNOWN")

Button(root, text="Arm", command=async_handler(arm),width=3).grid(row=4,column=2,rowspan=1,columnspan=1)
Button(root, text="Disarm", command=async_handler(disarm),width=6).grid(row=4,column=2,rowspan=1,columnspan=2)

labelFlightModeText = StringVar()
labelFlightModeText.set("Flight Mode:")
FlightModeLabel = Label(root, textvariable=labelFlightModeText, height=1)
FlightModeLabel.grid(row=6,column=1,rowspan=1)

FlightModeText = Text(root, height=1, width = 15)
FlightModeText.grid(row = 6, column = 1, rowspan = 1,columnspan=2)
FlightModeText.insert(END,"UNKNOWN")

labelLandedStateText = StringVar()
labelLandedStateText.set("Landed State: ")
LandedStateLabel = Label(root, textvariable = labelLandedStateText, height = 1)
LandedStateLabel.grid(row=7,column=1)

LandedStateText = Text(root,height = 1, width = 15)
LandedStateText.grid(row = 7, column = 1, rowspan = 1, columnspan = 2)
LandedStateText.insert(END,"UNKNOWN")

Button(root, text="Hold", command=async_handler(change_hold_mode),width=3).grid(row=6,column=2,rowspan=1,columnspan=1)

labelAltText=StringVar()
labelAltText.set("Altitude AGL:")
altLabel=Label(root, textvariable=labelAltText, height=4)
altLabel.grid(row=8,column=1,rowspan=1)

altText = Text(root, height=1, width=30)
altText.grid(row=8,column=2,rowspan=1)
altText.insert(END,"0 for 0 m")


pxhOut = Text(
    root,
    height=20,
    width=100
)
pxhOut.grid(row=12,column=1,columnspan=4)
pxhOut.insert(END,"Drone state will be shown here..."+ '\n')
# pxhOut.config(state=DISABLED)

labelRadiusInText=StringVar()
labelRadiusInText.set("Radius: ")
labelRadiusInObj=Label(root, textvariable=labelRadiusInText, height=4)
labelRadiusInObj.grid(row=6,column=3,rowspan=1,columnspan=1)

# nhap ban kinh (radius)
defRadius = StringVar(root, value='10')
radiusIn = Entry(root, textvariable=defRadius)
radiusIn.grid(row=6,column=4,rowspan=1,columnspan=1)

labelAltInText=StringVar()
labelAltInText.set("Take-Off Altitude: ")
labelAltInObj=Label(root, textvariable=labelAltInText, height=4)
labelAltInObj.grid(row=1,column=3,rowspan=1,columnspan=1)

# nhap cao do (altitude)
defAlt = StringVar(root, value='5')
altIn = Entry(root, textvariable=defAlt)
altIn.grid(row=1,column=4,rowspan=1,columnspan=1)

Button(root, text="Take-Off", command=async_handler(takeoff),width=30).grid(row=2,column=3,rowspan=1,columnspan=2)
Button(root, text="Land Current Position", command=async_handler(land),width=30).grid(row=4,column=3,columnspan=2)
Button(root, text="Do A Circle", command=async_handler(do_a_circle),width=30).grid(row=7,column=3,columnspan=2)

labelNorthInText=StringVar()
labelNorthInText.set("North Position (m): ")
labelNorthInObj=Label(root, textvariable=labelNorthInText, height=1)
labelNorthInObj.grid(row=2,column=6,rowspan=1,columnspan=1)
defNorth = StringVar(root, value='0')
NorthIn = Entry(root, textvariable=defNorth)
NorthIn.grid(row=2,column=7,rowspan=1,columnspan=1)

labelEastInText=StringVar()
labelEastInText.set("East Position (m): ")
labelEastInObj=Label(root, textvariable=labelEastInText, height=1)
labelEastInObj.grid(row=3,column=6,rowspan=1,columnspan=1)
defEast = StringVar(root, value='0')
EastIn = Entry(root, textvariable=defEast)
EastIn.grid(row=3,column=7,rowspan=1,columnspan=1)

labelDownInText=StringVar()
labelDownInText.set("Down Position (m): ")
labelDownInObj=Label(root, textvariable=labelDownInText, height=1)
labelDownInObj.grid(row=4,column=6,rowspan=1,columnspan=1)
defDown = StringVar(root, value='0')
DownIn = Entry(root, textvariable=defDown)
DownIn.grid(row=4,column=7,rowspan=1,columnspan=1)

labelYawInText=StringVar()
labelYawInText.set("Yaw Degree: ")
labelYawInObj=Label(root, textvariable=labelYawInText, height=1)
labelYawInObj.grid(row=5,column=6,rowspan=1,columnspan=1)
defYaw = StringVar(root, value='0')
YawIn = Entry(root, textvariable=defYaw)
YawIn.grid(row=5,column=7,rowspan=1,columnspan=1)

HeadingText = Text(root, height=1, width = 37)
HeadingText.grid(row = 6, column = 6, rowspan = 1,columnspan=2)
HeadingText.insert(END,"Heading: UNKNOWN")

NedPosText = Text(root, height=1, width = 37)
NedPosText.grid(row = 7, column = 6, rowspan = 1,columnspan=2)
NedPosText.insert(END,"UNKNOWN")

Button(root, text="Send NED Position", command=async_handler(offboard),width=30).grid(row=8,column=6,rowspan=1,columnspan=2)

Button(root, text="Forward", command=async_handler(forward),width=6).grid(row=2,column=10,rowspan=1,columnspan=1)
Button(root, text="Back", command=async_handler(back),width=6).grid(row=4,column=10,rowspan=1,columnspan=1)
Button(root, text="Left", command=async_handler(left),width=6).grid(row=3,column=9,rowspan=1,columnspan=1)
Button(root, text="Right", command=async_handler(right),width=6).grid(row=3,column=11,rowspan=1,columnspan=1)
Button(root, text="Clockwise", command=async_handler(clockwise),width=12).grid(row=1,column=9,rowspan=1,columnspan=1)
Button(root, text="Count-Clockwise", command=async_handler(count_clockwise),width=12).grid(row=1,column=11,rowspan=1,columnspan=1)
labelBodyStepText=StringVar()
labelBodyStepText.set("Body Step")
labelBodyStepObj=Label(root, textvariable=labelBodyStepText, height=1)
labelBodyStepObj.grid(row=3,column=10,rowspan=1,columnspan=1)

labelBodyStepInText=StringVar()
labelBodyStepInText.set("Body Step (m): ")
labelBodyStepInObj=Label(root, textvariable=labelBodyStepInText, height=1)
labelBodyStepInObj.grid(row=6,column=9,rowspan=1,columnspan=1)
defBodyStep = StringVar(root, value='1')
BodyStepIn = Entry(root, textvariable=defBodyStep, width=15)
BodyStepIn.grid(row=6,column=10,rowspan=1,columnspan=1)

#cai toc do max (m/s)

labelmaxSpeedInText=StringVar()
labelmaxSpeedInText.set("Max Speed (m/s): ")
labelmaxSpeedInObj=Label(root, textvariable=labelmaxSpeedInText, height=1)
labelmaxSpeedInObj.grid(row=8,column=9,rowspan=1,columnspan=1)
defmaxSpeed = StringVar(root, value='1')
maxSpeedIn = Entry(root, textvariable=defmaxSpeed, width=15)
maxSpeedIn.grid(row=8,column=10,rowspan=1,columnspan=1)
async_mainloop(root)
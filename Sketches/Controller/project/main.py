from time import sleep_ms, time_ns
from machine import I2C, Pin
from machine_i2c_lcd import I2cLcd
import network
from sys import exit
from microdot import Microdot, send_file, abort, redirect
from microdot.websocket import with_websocket
from microdot.cors import CORS
import uasyncio as asyncio
from urllib.parse import urlparse, parse_qs
from rgb_led import rgb_led
from menu import menu
from rotary import Rotary
import json


app = Microdot()
cors = CORS(app, allowed_origins=['http://127.0.0.1:8000'], allow_credentials=True) # for web-debugging

# mode and statemachine
systemMode = 0 # 0:config 1:game
systemStateMachine = 0 # 0:noGame 1:awaitName 2:awaitStart 3:awaitEnd
sensorState = 0 # 0:all ok -10:config error -1:sensor disconnected

penalty = 10 # penalty per interrupt in seconds
threshold = 240

lastList = []
topList = []

sensors = []
sensorsCount = 0
class Sensor:
    address = 0
    deviceType = 0
    deviceMode = 0
    conStatus = 1
    value = -1
    skip = 0
    def __init__ (self, address):
        self.address = address
        self.setMode(0)
        sleep_ms(200)
        self.deviceType = int.from_bytes(i2c.readfrom(self.address, 1, True), 'big')
    def __str__(self):
        return dict(address = hex(self.address), deviceType = self.deviceType, deviceMode = self.deviceMode, conStatus = self.conStatus, value = self.value)
    def setMode(self, mode):
        i2c.writeto(self.address, bytearray([0b00010000 + mode]), True)
        self.deviceMode = mode
    def setType(self, type):
        i2c.writeto(self.address, bytearray([0b00100000 + type]), True)
        self.deviceType = type
    def setThreshold(self, threshold):
        i2c.writeto(self.address, bytearray([0b01000000 + (threshold >> 8)]), True)
        i2c.writeto(self.address, bytearray([threshold & 0b0000000011111111]), True)
        #i2c.writeto(self.address, bytearray([0b01001111]), True)
        #i2c.writeto(self.address, bytearray([0b10101010]), True)
    def readValue(self):
        try:
            data = i2c.readfrom(self.address, 2, True)
            self.conStatus = 1
            self.value = int.from_bytes(data, 'big')
        except OSError as error:
            self.error()
    def readHit(self):
        try:
            data = i2c.readfrom(self.address, 1, True)
            self.conStatus = 1
            self.value = int.from_bytes(data, 'big')
        except OSError as error:
            self.error()
    def error(self):
        self.conStatus -= 1
        if self.conStatus < -5:
            global sensorState
            if sensorState == 0:
                sensorState = -1
            self.skip = 1
    def getHit(self):
        self.readHit()
        return self.value

def init():
    # led config
    global led
    led = rgb_led(7,8,9,500)
    led.color("yellow")
    global ledb
    ledb = machine.Pin("LED", machine.Pin.OUT)
    ledb.off()
    # buzzer
    global buzzer
    buzzer = Pin(5, Pin.OUT)
    buzzer.off()
    # init LCD
    lcd = I2cLcd(I2C(1, sda=14, scl=15, freq=100000), 0x27, 2, 16)
    # init Menu
    global menu
    menu = menu(lcd)
    menu.add_entry("", "Laser-Parcour")
    menu.show()
    # open_drain
    open_drain = Pin(19, Pin.IN, Pin.PULL_UP)
    def open_drain_handler(pin):
        tempTime = time_ns()
        global systemStateMachine
        if systemStateMachine == 2 and sensorState == 0:
            global startSensor
            if startSensor.getHit() == 1:
                asyncio.create_task(beep())
                global startTime
                startTime = tempTime
                systemStateMachine = 3
                led.color("green")
        elif systemStateMachine == 3:
            global endSensor
            if endSensor.getHit() == 1:
                asyncio.create_task(beep())
                global endTime
                endTime = tempTime
                global startTime
                deltatime = (endTime - startTime)*1e-9
                global interrupted
                totalTime = deltatime + interrupted * penalty
                global lastList
                lastList.append({'name': nextName, 'time': totalTime, 'delta': deltatime, 'interrupted': interrupted})
                if len(lastList) > 10:
                    lastList.pop(0)
                global topList
                topList.append({'name': nextName, 'time': totalTime, 'delta': deltatime, 'interrupted': interrupted})
                def sortkey(e):
                    return e['time']
                topList.sort(key=sortkey)
                if len(topList) > 10:
                    topList.pop(-1)
                systemStateMachine = 1
                led.color("blue")
    open_drain.irq(trigger=Pin.IRQ_FALLING, handler=open_drain_handler)
    # roraty encoder config
    rotary = Rotary(11, 13, 12)
    def rotary_changed(change):
        if change == Rotary.ROT_CW:
            menu.rot_cw()
        elif change == Rotary.ROT_CCW:
            menu.rot_ccw()
        elif change == Rotary.SW_PRESS:
            menu.rot_go()
        elif change == Rotary.SW_RELEASE:
            pass
    rotary.add_handler(rotary_changed)
    # init bus
    global i2c
    i2c = I2C(0,sda=Pin(16),scl=Pin(17),freq=4800) # set speed form 100000 to xxx
    scanBus()
    global lostSensorsMsg
    lostSensorsMsg = 0

def scanBus():
    global sensors
    global startSensor
    global endSensor
    global sensorState
    if sensorState == -10:
        sensorState = 0
    sensors = []
    startSensor = None
    endSensor = None
    print('Scaning Sensor Bus...')
    global i2c
    foundAdresses = i2c.scan()
    if len(foundAdresses) == 0:
        print('No Sensor found!')
    else:
        print(f'{len(foundAdresses)} Sensors found')       
        for address in foundAdresses:
            sensor = Sensor(address)
            print(str(sensor))
            sensors.append(sensor)
            if sensor.deviceType == 2:
                if startSensor != None:
                    print("more then one startSensor found")
                    sensorState = -10
                startSensor = sensor
            elif sensor.deviceType == 3:
                if endSensor != None:
                    print("more then one endSensor found")
                    sensorState = -10
                endSensor = sensor
    if startSensor == None:
        print("no startSensor found")
        sensorState = -10
    if endSensor == None:
        print("no endSensor found")
        sensorState = -10       
    for sensor in sensors:
        sensor.setMode(1)
        sensor.skip = 0
    for sensor in sensors:
        sensor.setThreshold(threshold)
        
def createWap():
    rp2.country('DE')
    wap = network.WLAN(network.AP_IF)
    wap.config(essid='Laser-Parkour', password='nN7o1xt3')
    wap.active(True)   

def checkForWap():
    return network.WLAN(network.AP_IF).active()

def netInfo():
    netConfig = network.WLAN(network.AP_IF).ifconfig()
    menu.add_entry(netConfig[0], "Network:", "IP:")
    menu.add_entry(netConfig[1], "Network:", "Mask:")
    menu.add_entry(netConfig[2], "Network:", "Gateway:")
    menu.add_entry(netConfig[3], "Network:", "DNS:")
    
@app.route('/')
async def index(request):
    return send_file('index.html')

@app.route('/script.js')
async def index(request):
    return send_file('script.js')

@app.route('/style.css')
async def index(request):
    return send_file('style.css')

@app.route('/action/rescan')
async def index(request):
    if systemMode == 0 or systemStateMachine <= 2:
        scanBus()
        return'scanning'
    else:
        return abort(428, 'system in wrong mode, unable to scan')

@app.route('/action/name')
async def index(request):
    global systemStateMachine
    global sensorState
    #print(parse_qs(urlparse('?' + request.query_string).query))
    if systemStateMachine == 1 and sensorState == 0:
        global nextName
        nextName = parse_qs(urlparse('?' + request.query_string).query)['name'][0]
        systemStateMachine = 2
        led.color("cyan")
        global interrupted
        interrupted = 0
        return nextName
    else:
        return abort(428, 'system not ready')

@app.route('/action/threshold')
async def index(request):
    global sensorState
    if systemMode == 0:
        global threshold
        if request.query_string:
            data = int(parse_qs(urlparse('?' + request.query_string).query)['threshold'][0])
            if data <= 1023 and data >= 0:
                threshold = data
            for sensor in sensors:
                sensor.setThreshold(threshold)
            print(threshold)
        return str(threshold)
    else:
        return abort(428, 'system not ready')

@app.route('/action/penalty')
async def index(request):
    global sensorState
    if systemMode == 0:
        global penalty
        if request.query_string:
            data = float(parse_qs(urlparse('?' + request.query_string).query)['penalty'][0])
            if data <= 240 and data >= 0:
                penalty = data
            print(penalty)
        return str(penalty)
    else:
        return abort(428, 'system not ready')

@app.route('/action/startgame')
async def index(request):
    global systemMode
    global sensorState
    if systemMode == 0 and sensorState == 0:
        systemMode = 1
        for sensor in sensors:
            sensor.setMode(2)
        global systemStateMachine
        systemStateMachine = 1
        led.color("blue")
        return redirect('/?m=game', status_code=303)
    else:
        return redirect('/?m=setup', status_code=303)
        
@app.route('/action/setup')
async def index(request):
    global systemMode
    global systemStateMachine
    global sensors
    led.color("pink")
    if systemMode == 1:
        for sensor in sensors:
            sensor.setMode(1)
        systemMode = 0
    systemStateMachine = 0
    return  json.dumps(systemMode)

@app.route('/API/playerlist')
async def index(request):
    global lastList
    global topList
    data = {"topList": topList, "lastList": lastList}
    return  json.dumps(data)

@app.route('/API/sensors')
@with_websocket
async def api(request, ws):
    global systemMode
    global systemStateMachine
    global sensors
    while True:
        if systemMode == 1:
            led.color("pink")
            for sensor in sensors:
                sensor.setMode(1)
            systemMode = 0
        systemStateMachine = 0
        response = '['
        for sensor in sensors:
            response += str(sensor).replace("'", '"') + ','
        response = response[:-1]
        response += ']'
        await ws.send(response)
        await ws.receive()
        await asyncio.sleep(0.5)


@app.route('/API/game')
@with_websocket
async def api(request, ws):
    global systemMode
    global systemStateMachine
    global sensorState
    while True:
        data = {}
        data['systemMode'] = systemMode
        data['systemStateMachine'] = systemStateMachine
        data['sensorState'] = sensorState
        response = json.dumps(data)
        await ws.send(response)
        await ws.receive()
        await asyncio.sleep(0.5)

    
async def startWeb():
    await app.run(debug=True)

async def beep():
    global buzzer
    buzzer.on()
    await asyncio.sleep_ms(200)
    buzzer.off()
      
async def main() :
    asyncio.create_task(startWeb())
    led.color("pink")
    global systemMode
    global sensors
    global systemStateMachine
    global systemMode
    global sensorState
    global lostSensorsMsg
    while True:
        ledb.on()
        if systemMode == 1:
            for sensor in sensors:
                if sensor.skip == 0:
                    sensor.readHit()
                    if sensor.value == 1 and sensor.deviceType == 1 and systemStateMachine == 3:
                        global interrupted
                        interrupted += 1
                        print("interrupted " + str(hex(sensor.address)))
                        asyncio.create_task(beep())
            if sensorState == -1:
                led.color("orange")
                if lostSensorsMsg == 0:
                    print("lost sensors")
                    lostSensorsMsg = 1
        elif systemMode == 0:
            for sensor in sensors:
                sensor.readValue()
        if systemStateMachine <= 2 or systemMode == 0:
            if sensorState == -1:
                led.color("red")
                if lostSensorsMsg == 0:
                    print("lost sensors")
                    lostSensorsMsg = 1
                global sensorsCount
                if len(sensors) > sensorsCount:
                    sensorsCount = len(sensors)
                #print(sensorsCount)
                global i2c
                foundAdresses = i2c.scan()
                #print(len(foundAdresses))
                if sensorsCount == len(foundAdresses):
                    led.color("yellow")
                    scanBus()
                    if len(sensors) == sensorsCount:
                        if systemMode == 1:
                            for sensor in sensors:
                                sensor.setMode(2)
                                sensor.skip = 0
                        sensorState = 0
                        lostSensorsMsg = 0
                        if systemStateMachine == 1:
                            led.color("blue")
                        elif systemStateMachine == 2:
                            led.color("cyan")
                        if systemMode == 0:
                            led.color("pink")
        ledb.off()
        await asyncio.sleep(0.1)   # Sleep for 0.1 seconds
    
if __name__=="__main__":
    if not checkForWap():
        createWap()
    init()
    netInfo()
    try:
        asyncio.run(main())
    finally:
        asyncio.new_event_loop()

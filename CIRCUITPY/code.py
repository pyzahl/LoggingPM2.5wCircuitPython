# Feather ESP32 S2 SSSM
# Welcome to CircuitPython 8 :)

import board
import gc
import time
import math
from digitalio import DigitalInOut, Direction, Pull
from analogio import AnalogIn
#from audioio import WaveFile, AudioOut
import touchio
#import pulseio
import displayio
#from adafruit_st7789 import ST7789
from adafruit_display_shapes.rect import Rect

import busio
#import adafruit_vl53l0x
import ipaddress
import adafruit_tsl2591
from adafruit_lc709203f import LC709203F, PackSize

import ssl
import socketpool
import wifi
import adafruit_ntp
import rtc
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import adafruit_requests

from adafruit_pm25.i2c import PM25_I2C

import random
#from rainbowio import colorwheel
from adafruit_esp32s2tft import ESP32S2TFT

from rainbowio import colorwheel
import neopixel

gc.collect()   # make some rooooom

DipSW = {
	'1': DigitalInOut(board.A2),  ## Read/Write (DIP on := GND [==Logic False]) => recording to logfile / no recording
	'2': DigitalInOut(board.A3),  ## MQTT Logging On/Off  *** disabled Use I2C Sensor
	'3': DigitalInOut(board.A4),  ## FireCapture prints/Full report prints to console
	'4': DigitalInOut(board.A5),  ## WiFi ON / OFF
	}

DipSWinfoClosed = { ## Logic "False" as DIP=ON is pin on GND
        '1': 'Record to Logfile',
        '2': 'MQTT IoT Transmit',
        '3': 'FC Print',
        '4': 'WiFi',
        }

DipSWinfoOpen = { ## Logic "True" = OPEN/OFF
        '1': 'No Logging',
        '2': 'No MQTT',
        '3': 'Long Status Print',
        '4': 'WiFi Off',
        }

# DIP SWITCH INFO
print ('Note: DIP ON (=GND) := False')
for pin in sorted(DipSW):
	DipSW[pin].switch_to_input(pull=Pull.UP)
        if DipSW[pin].value == False:
                info = DipSWinfoClosed[pin]
        else:
                info = DipSWinfoOpen[pin]
	print ('Dip SW', pin, DipSW[pin].value, info)

PinPhotoOK = DigitalInOut(board.D5)
PinPhotoOK.direction = Direction.OUTPUT


# Create sensor object, using the board's default I2C bus.
battery_mon = False
if battery_mon:
        battery_monitor = LC709203F(board.I2C())

        # Update to match the mAh of your battery for more accurate readings.
        # Can be MAH100, MAH200, MAH400, MAH500, MAH1000, MAH2000, MAH3000.
        # Choose the closest match. Include "PackSize." before it, as shown.
        battery_monitor.pack_size = PackSize.MAH400

        print("Battery Percent: {:.2f} %".format(battery_monitor.cell_percent))
        print("Battery Voltage: {:.2f} V".format(battery_monitor.cell_voltage))
    
reset_pin = None 
pm25 = PM25_I2C(board.I2C(), reset_pin)
print("Found PM2.5 sensor, reading data...")

PM_LIST_ENV = ['pm10 env', 'pm25 env', 'pm100 env' ]
PM_LIST_STD = ['pm10 standard', 'pm25 standard', 'pm100 standard' ]
PM_LIST = [ 'particles 03um', 'particles 05um', 'particles 10um', 'particles 25um', 'particles 50um', 'particles 100um' ]


def read_pm25_sensor(verbose=False, Init=False):
    try:
        aqdata = pm25.read()
        if Init:
                print("Initial AQD readings...")
                time.sleep(1)
                print(" *")
                aqdata = pm25.read()
                time.sleep(1)
                print(" *")
                aqdata = pm25.read()
                time.sleep(1)
                print(" *")
                aqdata = pm25.read()
                print(" *")
        #print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        return
    # {'pm10 env': 0, 'pm100 env': 0, 'pm100 standard': 0, 'particles 03um': 105, 'pm25 standard': 0, 'particles 10um': 0, 'pm10 standard': 0, 'pm25 env': 0, 'particles 05um': 29, 'particles 25um': 0, 'particles 100um': 0, 'particles 50um': 0}


    print(" OK \n")

    if verbose:
        print()
        print("Concentration Units (standard)")
        #print("---------------------------------------")
        print("PM1.0: {:4d} PM2.5: {:4d} PM10: {:4d}".format(aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"]))
        print("Concentration Units (environmental)")
        #print("---------------------------------------")
        print("PM1.0: {:4d} PM2.5: {:4d} PM10: {:4d}".format(aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"]))
        #print("---------------------------------------")
        print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
        print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
        print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
        print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
        print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
        print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
        #print("---------------------------------------")   
    return aqdata

if 0: 
  while True:
    time.sleep(1)
    read_pm25_sensor(True)

# Init Reads
aqd = read_pm25_sensor(True, True)
aqd010_iir = aqd['pm10 standard']
aqd025_iir = aqd['pm25 standard']
aqd100_iir = aqd['pm100 standard']
MQTTint = 60
qn = 1.0/MQTTint
qo = 1.0-qn

# Get wifi details and more from a secrets.py file
try:
	from secrets import secrets
except ImportError:
	print("WiFi secrets are kept in secrets.py, please add them there!")
	raise

if DipSW['2'].value == False:  ## '4' WiFi
        print('Connecting WiFi...')
        print("Connecting to %s" % secrets["ssid"])
        wifi.radio.connect(secrets["ssid"], secrets["password"])
        print("Connected to %s!" % secrets["ssid"])

        pool = socketpool.SocketPool(wifi.radio)
        ntp = adafruit_ntp.NTP(pool, tz_offset=5)

        print('try: NTP clock sync...')

        #print(ntp.datetime)
        #print(time.localtime())

        # NOTE: This changes the system time so make sure you aren't assuming that time
        print('try: Sync system time to NTP...')
        rtc.RTC().datetime = ntp.datetime
        tmnow = time.localtime()
        #struct_time(tm_year=2022, tm_mon=7, tm_mday=22, tm_hour=4, tm_min=56, tm_sec=28, tm_wday=4, tm_yday=203, tm_isdst=-1)
        print('Start Timestamp: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d}'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec))
else:
        print ('WiFi OFF, no NTP auto time set.')
        tmnow = time.localtime()
        print ('System Start Timestamp: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d}'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec))

  
# Setup a feed named 'photocell' for publishing to a feed
#sssm_time_feed = "feeds/sssm/time"
#sssm_arcs_feed = "feeds/sssm/arcs"
#sssm_ref_feed  = "feeds/sssm/refint"
#sssm_lux_feed  = "feeds/sssm/lux"
#sssm_bat_feed  = "feeds/sssm/battery"
aqd_pm010_feed = "pyzahl2/feeds/aqd.pm010"
aqd_pm025_feed = "pyzahl2/feeds/aqd.pm025"
aqd_pm100_feed = "pyzahl2/feeds/aqd.pm100"

## test adafruit.io -- very limited rate and points (30/min total)
#sssm_time_feed = "pyzahl2/feeds/sssm.time"
#sssm_arcs_feed = "pyzahl2/feeds/sssm.arcs"
#sssm_ref_feed  = "pyzahl2/feeds/sssm.refint"
#sssm_lux_feed  = "pyzahl2/feeds/sssm.lux"
#sssm_bat_feed  = "pyzahl2/feeds/sssm.battery"

# Setup a feed named 'onoff' for subscribing to changes
#onoff_feed = "/feeds/info"
def connect(mqtt_client, userdata, flags, rc):
    print("Connected to MQTT Broker!")
    print("Flags: {0}\n RC: {1}".format(flags, rc))


def disconnect(mqtt_client, userdata, rc):
    print("Disconnected from MQTT Broker!")


def subscribe(mqtt_client, userdata, topic, granted_qos):
    print("Subscribed to {0} with QOS level {1}".format(topic, granted_qos))


def unsubscribe(mqtt_client, userdata, topic, pid):
    print("Unsubscribed from {0} with PID {1}".format(topic, pid))


def publish(mqtt_client, userdata, topic, pid):
    print("Published to {0} with PID {1}".format(topic, pid))


def message(client, topic, message):
    print("New message on topic {0}: {1}".format(topic, message))


if DipSW['2'].value == False:
        print ('MQTT Client Configuration:')
        pool = socketpool.SocketPool(wifi.radio)

        mqtt_client = MQTT.MQTT(
                broker=secrets["broker"],
                port=secrets["port"],
                username=secrets["aio_username"],
                password=secrets["aio_key"],
                socket_pool=pool,
                #is_ssl=True,
                #ssl_context=ssl.create_default_context(),
        )

        mqtt_client.on_connect = connect
        mqtt_client.on_disconnect = disconnect
        mqtt_client.on_subscribe = subscribe
        mqtt_client.on_unsubscribe = unsubscribe
        #mqtt_client.on_publish = publish
        mqtt_client.on_message = message

	print("MQTT: Attempting to connect to {}::{}".format(mqtt_client.broker, mqtt_client.port))
	try:
		mqtt_client.connect()
		mqtt_client_connected = True
	except Exeption as e: # (OSError, RuntimeError)
		print ('Connect error: ',e)
		mqtt_client_connected = False
	        print ('MQTT disabled now.')
else:
        print ('MQTT disabled.')

logging = False
log_count=0
mqtt_count=0
mqtt_p=0

if DipSW['1'].value == False:
	try:
	    with open("/aqd.log", "a") as light_log:
		light_log.write('# Log Start: {:04d} {:02d} {:02d}  {:02d}:{:02d}:{:02d} ** TIMESTAMP T0={:d}\n'.format(tmnow.tm_year, tmnow.tm_mon, tmnow.tm_mday, tmnow.tm_hour, tmnow.tm_min, tmnow.tm_sec, time.time()))
		light_log.flush()
                light_log.close()
		logging = True
	except OSError as e:  # When the filesystem is NOT writable by CircuitPython...
		print('Can not access drive:', e)


### ESP32-S2 240MHz Color 1.14" IPS TFT with 240x135 pixels ST7789
esp32s2tft = ESP32S2TFT(
    default_bg=0x000000,
    scale=1,
)


# Create the labels
status = esp32s2tft.add_text(
	text='SSSMv0',
	text_position=(0, 10),
	text_scale=3,
	text_color=0xFF00FF
)

if battery_mon:
        battery = esp32s2tft.add_text(
	        text='{:.1f} %'.format(battery_monitor.cell_percent),
	        text_position=(240, 10),
	        text_anchor_point=(1.0, 0.5),
	        text_scale=1,
	        text_color=0x00FF00
        )

Linfo_last = "ReCal:"
Minfo_last = "BOOT0 Button"
info_label = esp32s2tft.add_text(
	text="ReCal: BOOT0 Button",
	line_spacing=1.0,
	text_position=(240, 20),
	text_anchor_point=(1.0, 0.5),
	text_scale=1,
	text_color=0x606060,
)

reading_last = 'Record: boot+A3=GND'

reading = esp32s2tft.add_text(
	text=reading_last,
	text_position=(0, 40),
	text_scale=2,
	text_anchor_point=(0.0, 0.5),
	text_color=0xFFFF00,
)

if 0:
	reading_aux_last = 'AUX READING'

	reading_aux = esp32s2tft.add_text(
		text=reading_aux_last,
		text_position=(0, 65),
		text_scale=2,
		text_anchor_point=(0.0, 0.5),
		text_color=0xFFFF00,
	)

	reading_arc_last = 'ARC READING'

	reading_arc = esp32s2tft.add_text(
		text=reading_arc_last,
		text_position=(0, 90),
		text_scale=2,
		text_anchor_point=(0.0, 0.5),
		text_color=0xFFFF00,
	)

if 1:
	GYMAX = 70
	GYMAX1=GYMAX-1
	graph_bitmap = displayio.Bitmap(240, GYMAX, 6)
	color_palette = displayio.Palette(6)
	color_palette[0] = 0x000000
	color_palette[1] = 0xFF3000
	color_palette[2] = 0x00FF00
	color_palette[3] = 0x0000FF
	color_palette[4] = 0x330000
	color_palette[5] = 0x003300

	gr_sprite = displayio.TileGrid(graph_bitmap,
		                       pixel_shader=color_palette,
		                       x=0, y=55)
		                       
	esp32s2tft.splash.append(gr_sprite)


#NN=32
#for i in range(NN):
#	esp32s2tft.splash.append(Rect(i*8, 100, 4, 20, fill=0x00FF00))

esp32s2tft.display.show(esp32s2tft.splash)

### TSL Light Sensor I2C
i2c = board.I2C()

print ('Checking for LUX sensor TSL2591')
try:
	sensor = adafruit_tsl2591.TSL2591(i2c)
	sensor.gain = adafruit_tsl2591.GAIN_LOW
	sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_100MS
	time.sleep(1.0)
	lux_read = True
        print ('... found TSL2591')
except ValueError as e:
	print(e)
	lux_read = False
        print ('... none')

# You can optionally change the gain and integration time:
# sensor.gain = adafruit_tsl2591.GAIN_LOW  #    1x gain
# sensor.gain = adafruit_tsl2591.GAIN_MED  #   25x gain, the default
# sensor.gain = adafruit_tsl2591.GAIN_HIGH #  428x gain
# sensor.gain = adafruit_tsl2591.GAIN_MAX  # 9876x gain

# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_100MS # 100ms, default
# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS # 200ms
# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_300MS # 300ms
# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_400MS # 400ms
# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_500MS # 500ms
# sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_600MS # 600ms


# Measure Pulse Length
#NMEASURE  = 128
#NMEASURE2 = 512
#pulses = pulseio.PulseIn(board.D10, maxlen=NMEASURE)
#time.sleep (1.0)

####PASS vl53l0x.VL53L0X

# Built in red LED
#led = DigitalInOut(board.LED)
#led.direction = Direction.OUTPUT
#led.value = False


# Analog input on A1
analog0in = AnalogIn(board.A0)
analog1in = AnalogIn(board.A1)

######################### HELPERS ##############################

# Helper to convert analog input to voltage
def getVoltage(pin):
    return (pin.value * 3.3) / 65536

def Voltage(value):
    return (value * 3.3) / 65536

def mVoltage(value):
    return (value * 3.3) / 65.536

## SSSM analog test -- NOTE: we do have only FLOAT here (30-bit wide floating point) or 32bit int
def SSSM_analog():
	I=0
	RMS=0
        ##** Offset Calibration only
        ##R=0
        ##U0=0
        ##U1=1
        ##**
	for i in range(1024):
                #                      Center Value  CZ     Py      Ideal
		u0 = analog0in.value - 32199        #32199 #33025  #32768  # offset corrected
		u1 = analog1in.value - 32956        #32965 #33104  #32768  # offset corrected
		I = I + u0
		RMS= RMS + u1*u1
                ##** Calib only
                ##R = R + u1
		##U0 = U0 + analog0in.value
                ##U1 = U1 + analog1in.value
                ##**
        ##** CALIBRATION MODE PRINT READINGS:
        ##print (U0>>10, U1>>10, I, R, RMS, analog0in.value, analog1in.value)
        ##U0=U0>>10
        ##U1=U1>>10
        ##print (U0, U1, mVoltage(U0), mVoltage(U1))
        ##**

        ref = I/32.0   # I>>10 = I/32/32
	if ref < 1:
		ref = 1
	rms = math.sqrt(RMS)  ## sqrt(RMS>>10) = sqrt(RMS)/32
	# arcs=1886.79/425.5*rms/ref ## 425.5 is the AC 400Hz BW signal gain vs ref singal. 1886.79 is the conversion factor to arcs
	# 1886.79/425.5 = 0.13857153349
	# (rms / 32) / (ref / 32)
	arcs = 4.434289*rms/ref
	if arcs > 20: # clip
		arcs=20.0
	return u0, u1, ref/32, rms/32, arcs

def SSSM_test():
	nowns = time.monotonic_ns()
	startns = nowns
	while True:
		nowns = time.monotonic_ns()
		u0, u1, ref, rms, arcs = SSSM_analog()
		reading_last = 'A0 {:4.2f}V A1 {:4.2f}V'.format(Voltage(u0), Voltage(u1))
		reading_aux_last = 'I {:5.3f} RMS {:5.3f}'.format(Voltage(ref), Voltage(rms))
		reading_arc_last = 'ARCs {:5.2f}'.format(arcs)
		
		print (reading_last+', '+reading_aux_last)
		esp32s2tft.set_text(
			reading_last, reading
		)
		esp32s2tft.set_text(
			reading_aux_last, reading_aux
		)
		esp32s2tft.set_text(
			reading_arc_last, reading_arc
		)

                if battery_mon:
		        esp32s2tft.set_text(
			        '{:.1f}s Bat {:.1f} %'.format((nowns-startns)/1e9, battery_monitor.cell_percent), battery
		        )

#SSSM_test()


## LUX Sensor Support

def LUXsensor_read():
	try:
		lux = sensor.lux
		visible = sensor.visible
		infrared = sensor.infrared
		full_spectrum = sensor.full_spectrum
		mode = 'S'
	except RuntimeError as e:
		print ('*S* Sensor Overexposed!', e)
		lux = 999.1
		visible = 1000000000
		mode = 'S*OV'
	if lux < 80.0: # switch to Low Light mode
		mode = 'L'
		sensor.gain = adafruit_tsl2591.GAIN_MED  # 9876x gain
		sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_200MS # 100ms
		time.sleep(0.5)
		try:
			lux = sensor.lux
			visible = sensor.visible
			infrared = sensor.infrared
			full_spectrum = sensor.full_spectrum
		except RuntimeError as e:
			print ('*L* Sensor Overexposed!', e)
			lux = 999.2
			visible = 1000000000
			mode = 'L*OV'
		time.sleep(0.5)
		if lux < 1.0: # switch to Night mode
			mode = 'N'
			sensor.gain = adafruit_tsl2591.GAIN_MAX  # 9876x gain
			sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_600MS # 600ms
			time.sleep(0.6)
			try:
				lux = sensor.lux
				visible = sensor.visible
				infrared = sensor.infrared
				full_spectrum = sensor.full_spectrum
			except RuntimeError as e:
				print ('*N* Sensor Overexposed!', e)
				lux = 999.3
				visible = 1000000000
				mode = 'N*OV'
			sensor.gain = adafruit_tsl2591.GAIN_LOW  # 9876x gain
			sensor.integration_time = adafruit_tsl2591.INTEGRATIONTIME_100MS # 100ms
			intns = 500000
			#time.sleep(0.5)
	else:
		intns = 500000
#		time.sleep(0.5)
	
	## http://hms.sternhell.at/lightwiki/index.php/Conversion_of_light_measurements
	#mag = -14.0 + 2.5 * math.log(lux)/2.302585092994046  ## needs log10  mag per arcsec
	if lux > 0.0:
		mag = -1.49 * math.log(lux) - 2.11 - 0.5 # vis mag
	else:
		mag = -99.0

	#print("Total light: {0}lux".format(lux))
	# You can also read the raw infrared and visible light levels.
	# These are unsigned, the higher the number the more light of that type.
	# There are no units like lux.
	# Infrared levels range from 0-65535 (16-bit)
	#infrared = sensor.infrared
	#print("Infrared light: {0}".format(infrared))
	# Visible-only levels range from 0-2147483647 (32-bit)
	#visible = sensor.visible
	#print("Visible light: {0}".format(visible))
	# Full spectrum (visible + IR) also range from 0-2147483647 (32-bit)
	#full_spectrum = sensor.full_spectrum
	#print("Full spectrum (IR + visible) light: {0}".format(full_spectrum))

	I = visible
	if ISum < 0 or arcs > 50.0 or esp32s2tft.peripherals.button:
		print("Init/A2 touched or Auto Resetting I0.")
		print('SEC------\tMODE\tLUX-----\tMAG--\tINT0--------\tINT---------\tarcs-\tarcs-\tRef--\tRMS---\tarcs-\t BAT%')

		I0 = I
		Iarr = []
		ISum = 0
		i=0
		for k in range(klen):
			Iarr.append(I)
			ISum = ISum+I
		ir = 0
		IrmsS = 0
		Irmsarr = []
		for k in range(rmslen):
			Irmsarr.append(0)
	ISum = ISum + I - Iarr[i]
	Iarr[i] = I
	i=i+1
	if i==klen:
		i=0
	I0 = ISum >> kshr
      
	dI = I0-I
	
	dI2 = dI*dI
	IrmsS = IrmsS + dI2 - Irmsarr[ir]
	Irmsarr[ir] = dI2
	ir=ir+1
	if ir == rmslen:
		ir=0
	dIrms = IrmsS >> rmsshr
	
	arms = math.sqrt(dIrms)/I0*1886.79  ## w = 1 arc sec, dI/<I> = 5.3 x 10e-4
	a = abs(dI/I0)*1886.79  ## w = 1 arc sec, dI/<I> = 5.3 x 10e-4

	#print(IrmsS, dIrms, arms)
	if a > 50:
		a=99.99 ## Limiter
	#long_reading = '{:d}\t{:s}\t{:8.4f}\t{:+5.1f}\t{:12d}\t{:12d}\t{:5.2f}\t{:5.2f}\t{:5.3f}\t{:6.3f}\t{:6.2f}\t{:5.2f}'.format(time.time(), mode, lux, mag, I0, I, a, arms, Voltage(ref), Voltage(rms), arcs, battery_monitor.cell_percent)
	#reading_last='{:s} {:.4f} {:.1f} {:.1f}'.format(mode, lux, mag, arcs)

	return lux, visible, infrared, full_spectrum




######################### MAIN LOOP ##############################

esp32s2tft.display.auto_refresh = False

#print('Test Log10(10): ', math.log(10)/2.302585092994046)

I0 = 0
i=0

klen = 16
kshr = 4
rmslen = 16
rmsshr = 4

ISum = -1
dI = 0
IrmsS = 0
arcs = 0.0

arcs_hist = []
lux_hist = []
nowns = time.monotonic_ns()
startns = nowns
intns = 100000

aqd_hist = []

# Start with header comment
long_reading = '#TIMSTAMP PM  1.0  2.5 10.0  AQD  0.3  0.5  1.0  2.5  5.0 10.0um'
reading_last = '***'
pm25r = 0
while True:
        if logging:
                if DipSW['1'].value == False:
                        try:
	                        with open("/aqd.log", "a") as light_log:
		                        light_log.write(long_reading+'\n')
		                        light_log.flush()
                                        light_log.close()
                                        log_count=log_count+1
		                        Linfo_last = "Log {:4d}".format(log_count)
	                except (Exception, OSError) as e:  # When the filesystem is NOT writable by CircuitPython...
		                Linfo_last = "Log ERR"
        else:
		Linfo_last = "#"
        
	if len(aqd_hist) >= 240:
		aqd_hist.pop(0)

	aqd = read_pm25_sensor(False)
	if aqd == None:
		print ('PM25 read error')
	else:
		aqd_reading = '{:4d} {:4d} {:4d} {:4d}'.format(aqd['particles 03um']-aqd['particles 05um'], aqd['particles 05um']-aqd['particles 10um'], aqd['particles 10um']-aqd['particles 25um'], aqd['particles 25um']-aqd['particles 50um'])
		reading_last = aqd_reading
                pm25r = aqd['pm25 standard']
                #TIMSTAMP PM  1.0  2.5 10.0  AQD  0.3  0.5  1.0  2.5  5.0 10.0um
                #946684809 PM   11   20   22  AQD    0    0    0    0    0    0
	        long_reading = '{:d} PM {:4d} {:4d} {:4d} AQD  {:4d} {:4d} {:4d} {:4d} {:4d} {:4d}'.format(time.time(), aqd['pm10 standard'], aqd['pm25 standard'], aqd['pm100 standard'], aqd['particles 03um'], aqd['particles 05um'], aqd['particles 10um'], aqd['particles 25um'], aqd['particles 50um'], aqd['particles 100um'])


		aqd010_iir = qo*aqd010_iir + qn*aqd['pm10 standard']
		aqd025_iir = qo*aqd025_iir + qn*aqd['pm25 standard']
		aqd100_iir = qo*aqd100_iir + qn*aqd['pm100 standard']

                
		#aqd_hist.append (aqd['particles 03um'])
		aqd_hist.append (aqd.copy())
	
	#print (aqd_hist)
	
	#for key in PM_LIST:
	#	print (key, aqd[key])

	x=0
	yp=0
	for yvals in (aqd_hist):
		# clear column
		for yc in range(0,GYMAX):
			if yc % 10 == 0:	
				graph_bitmap[x, yc] = 4	
			else:
				graph_bitmap[x, yc] = 0	
		#graph_bitmap[x, GYMAX-10] = 5 # green (1 arcs)	
		## AQD PM25 DATA
		ci=1
		for key in PM_LIST_STD:
			yi = GYMAX1 - (int(round(yvals[key]))%GYMAX)
			graph_bitmap[x, yi] = ci
			if yi < GYMAX1:
				graph_bitmap[x, yi+1] = ci
			#print(x,ci, key, yvals[key], yval, yi)
			ci=ci+1
		x=x+1
	
	status_reading = 'PM {:3d}'.format(pm25r)

	esp32s2tft.set_text(
		status_reading, status
	)

	esp32s2tft.set_text(
		reading_last, reading
	)

	esp32s2tft.set_text(
		Minfo_last + ' ' + Linfo_last, info_label
	)

        nowns = time.monotonic_ns()

	s=int((nowns-startns)/1e9)
        if battery_mon:
	        bat = battery_monitor.cell_percent
	        esp32s2tft.set_text(
		        '{:.1f}s Bat {:.1f} %'.format(s, bat), battery
	        )

	        esp32s2tft.set_text_color(
		        0xFF0000 if bat < 20 else 0x00FF00, battery
	        )



	esp32s2tft.set_text_color(
		0x00FF00 if arcs < 1.0 else 0xFF0000 if logging else 0xFF00FF, status
	)

	esp32s2tft.set_text_color(
		0xFF0000 if esp32s2tft.peripherals.button else 0x606060, info_label
	)
	#esp32s2tft.peripherals.led = esp32s2tft.peripherals.button
	#if esp32s2tft.peripherals.button:
	#	esp32s2tft.peripherals.neopixel[0] = colorwheel(random.randint(0, 255))

	esp32s2tft.display.refresh ()

	if DipSW['3'].value == False:
                print ('PM25: {:4d}'.format(pm25r))
        else:
                print (status_reading, long_reading)
                        
	

        # WiFi + MQTT?
        if DipSW['2'].value == False and mqtt_client_connected: ## ref signal > 100mV, arcs < 20
		try:
			esp32s2tft.peripherals.led = True
			# Poll the message queue
			#mqtt_client.loop()
			# Send a new message
                        #if s%3: # throttle rate
			print("MQTT publishing in {:d}".format(MQTTint - s%MQTTint))
                        if s%MQTTint < 2 and mqtt_p < 0:
                                mqtt_p = 0
                                
                        if aqd != None and mqtt_p >= 0:
			        print("MQTT publishing...")
                                mqtt_p=mqtt_p+1
                                if mqtt_p == 1:
			                mqtt_client.publish(aqd_pm010_feed, aqd010_iir)
                                elif mqtt_p == 2:
			                mqtt_client.publish(aqd_pm025_feed, aqd025_iir)
                                else:
			                mqtt_client.publish(aqd_pm100_feed, aqd100_iir)
                                        mqtt_p = -1
                                mqtt_count=mqtt_count+1
		        Minfo_last = "MQTT {:4d}:{:d}".format(mqtt_count, mqtt_p)
			esp32s2tft.peripherals.led = False
		except (Exception, OSError) as e: # (ValueError, OSError, EConnAborted, RuntimeError
			esp32s2tft.peripherals.led = False
			print("MQTT failed, retrying. ", e)
			Minfo_last = "WiFi reset attempt"
			try:
			        wifi.reset()
			        wifi.radio.connect(secrets["ssid"], secrets["password"])
			        mqtt_client.reconnect()
			except (Exception, OSError) as e:
				print("WiFi reset failed, retrying next loop. ", e)
        else:
		Minfo_last = "-"
		esp32s2tft.peripherals.led = False
                
        print (Minfo_last)

        if pm25r > 85:
                cwv=85
        else:
                cwv=85-pm25r
	esp32s2tft.peripherals.neopixel[0] = colorwheel(cwv)

        if arcs < 1.0:
		esp32s2tft.peripherals.led = True
		PinPhotoOK.value = True
	else:
		PinPhotoOK.value = False

        time.sleep(1)

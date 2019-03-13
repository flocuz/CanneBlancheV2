from object-detector import *

# Camera
from picamera import picamera
from time import sleep
# Text to speech
import pyttsx3
# Hokuyo lidar
import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

# Initialisation
engine = pyttsx3.init()
# Angle camera 62.2 x 48.8 degrees
camera = PiCamera()
camera.start_preview()
#Initialisation de la movidius
device = open_ncs_device()
graph = load_graph( device )

uart_port = '/dev/ttyACM0'
uart_speed = 19200
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)

while(True) :
    #faire la capture d'ecran
    camera.capture('image_pour_detection.jpg')

    img_draw = skimage.io.imread( 'image_pour_detection.jpg' )
    img = pre_process_image( img_draw )

    #traiter l'image avec object-detector
    tab = infer_image_v2( graph, img )

    #donnees lidar
    
    print(laser.laser_on())
    print(laser.get_single_scan())
    print(laser.get_version_info())
    print(laser.get_sensor_specs())
    print(laser.get_sensor_state())
    print(laser.set_high_sensitive())\
    print(laser.set_high_sensitive(False))
    print('---')
    print(laser.set_motor_speed(10))
    print('---')
    print(laser.set_motor_speed())
    print('---')
    print(laser.reset())
    print('---')
    print(laser.laser_off())

    #sorties
    
    #engine.say("I will speak this text")
    #engine.runAndWait()

camera.stop_preview()
engine.stop()
close_ncs_device( device, graph )

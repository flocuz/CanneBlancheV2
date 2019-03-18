from object_detector import *

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
# Camera
camera = PiCamera()
camera.start_preview()
# Movidius
device = open_ncs_device()
graph = load_graph( device )
# Laser
uart_port = '/dev/ttyACM0'
uart_speed = 19200
laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
print(laser.laser_on())
print(laser.get_version_info())
#print(laser.get_sensor_specs())
#print(laser.set_motor_speed(10))
#print(laser.set_high_sensitive(False))

ON = True
ANGLE_CAMERA = 62.2
RANGE_MIN = 3000

while(ON) :
    #faire la capture d'ecran
    camera.capture('image_pour_detection.jpg')

    img_draw = skimage.io.imread( 'image_pour_detection.jpg' )
    img = pre_process_image( img_draw )
    
    ( length, heigth, dim ) = img_draw.shape
    anglePixel = length / ANGLE_CAMERA

    #traiter l'image avec object-detector
    # Matrix[i][0] Score, Matrix[i][1] Label, Matrix[i][2] TopLeft, Matrix[i][3] BotRight,
    objectMatrix = infer_image_v2( graph, img )

    #donnees lidar
    
    scanTab = laser.get_single_scan()
    ( val, angle, indice ) = findMinValue( scanTab )
    
    #objet distance sync
    objectValid = []
    
    if( indice != -1 ):
        for object in objectMatrix:
            ( topX, topY ) = object[3]
            ( botX, botY ) = object[4]
            if ( ( topX * anglePixel ) >= angle and ( botX * anglePixel ) <= angle):
                objectValid.append(object)    
    
    #sorties   
    for object in objectValid:
        print(object[1])
        sleep(1000)
        #engine.say("I will speak this text")
    
    engine.runAndWait()
    print(laser.reset())

# Off
print(laser.laser_off())
camera.stop_preview()
engine.stop()
close_ncs_device( device, graph )

def findMinValue( tab ):
    min = 999999999
    ind = 0
    angle = -1
    indFound = -1
    
    for val in tab:
        # separe la chaine dans un tableau
        angleValue = val.split(': ')
        if( angleValue[1] < min and angleValue[1] > RANGE_MIN and angleValue[1] != -1 and angleValue[0] < ANGLE_CAMERA/2 and angleValue[0] > -ANGLE_CAMERA/2 ):
            angle = angleValue[0]
            min = angleValue[1]
            indFound = ind
        ind = ind +1
        
    return ( min, angle, indFound )

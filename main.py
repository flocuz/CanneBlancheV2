from object_detector import *

# Camera
from picamera import PiCamera
from time import sleep
from subprocess import call
# Text to speech
import pyttsx3
# Hokuyo lidar
import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

def findMinValue( tab ):
    minimum = 999999999
    ind = 0
    angle = -1
    indFound = -1

    angleMin = -1 * ANGLE_CAMERA * 0.5 
    angleMax = ANGLE_CAMERA * 0.5

    print(angleMin)
    print(angleMax)
    
    for angleTab in tab:
        # separe la chaine dans un tableau
        
        angleValue = [angleTab, tab[angleTab]]
        if( angleValue[1] < minimum and angleValue[1] <= RANGE_MIN and angleValue[1] > 50 and angleValue[0] < angleMax and angleValue[0] > angleMin ):
            angle = angleValue[0]
            minimum = angleValue[1]
            indFound = ind
        ind = ind +1

    return ( minimum, angle, indFound )

def getDirection( angle ) :
    angleMin = -1 * ANGLE_CAMERA * 0.5
    angleMax = ANGLE_CAMERA * 0.5
    res = ''
    if(angleMin <= angle and angle < angleMin + (ANGLE_CAMERA/3)):
        res = 'to your left'
    else:
        if (angleMin + (ANGLE_CAMERA / 3) <= angle and angle <= angleMax - (ANGLE_CAMERA / 3) ):
            res = 'in front of you'
        else:
            res = 'to your right'
    return res



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
OFFSET = 2.5

iteration = 0

while(iteration < 1) :
    #faire la capture d'ecran
    camera.capture('image_pour_detection.jpg')

    img_draw = skimage.io.imread( 'image_pour_detection.jpg' )
    


    #traiter l'image avec object-detector
    # Matrix[i][0] Score, Matrix[i][1] Label, Matrix[i][2] TopLeft, Matrix[i][3] BotRight,
    img = pre_process_image( img_draw )
    objectMatrix = infer_image_v2( graph, img_draw, img )
    print(objectMatrix)

    #donnees lidar
    
    scanTab = laser.get_single_scan()
    ( val, angle, indice ) = findMinValue( scanTab )

    ( heigth, length, dim ) = img_draw.shape
    anglePixel = ANGLE_CAMERA / length
    
    #objet distance sync
    objectValid = []
	
    print("Angle trouve pour le min")
    print(angle)
    print("Minimum : ")
    print(val)
    
    if( indice != -1 ):
        for object in objectMatrix:
            ( topY, topX ) = object[2]
            ( botY, botX ) = object[3]

            print("Pixels pour les box : topX = " + topX + " botX = " + botX)
            print("Angle pour les objets detectes : ["+  topX * anglePixel - (ANGLE_CAMERA / 2) + " , " + botX * anglePixel - (ANGLE_CAMERA / 2) + "]")
            print("Test condition : ")
            print(( topX * anglePixel - (ANGLE_CAMERA / 2))  - OFFSET)
            print(angle)
            print(( botX * anglePixel - (ANGLE_CAMERA / 2) )  + OFFSET)
            print(angle)


            




            if ( ( topX * anglePixel - (ANGLE_CAMERA / 2)) - OFFSET <= angle and ( botX * anglePixel - (ANGLE_CAMERA / 2) ) + OFFSET >= angle):
                print("a la con") 
                objectValid.append(object)    
    
    print("Objets valides")
    #sortie sonore (prononce "Attention 'objet' 'distance' meters"
    objects_str = ''
    for object in objectValid:
        print(object[1])
        objects_str = objects_str + object[1].split(': ')[1] + ' '

    sentence = 'Look out ' + objects_str + str(val / 1000) + ' meters ' + getDirection(angle)
    call(['espeak \'' + sentence + '\' 2>/dev/null'], shell=True)
    #call(['espeak \'Hello\' 2>/dev/null'], shell= True)
    #print(laser.reset())
    iteration = iteration + 1

    sleep(3)

# Off
print(laser.laser_off())
camera.stop_preview()
close_ncs_device( device, graph )



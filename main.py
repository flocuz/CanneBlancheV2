from object-detector import *

from picamera import picamera
from time import sleep
# Text to speech
import pyttsx3

# Initialisation
engine = pyttsx3.init()
# Angle camera 62.2 x 48.8 degrees
camera = PiCamera()
camera.start_preview()
#Initialisation de la movidius
device = open_ncs_device()
graph = load_graph( device )

while(True) :
    #faire la capture d'ecran
    camera.capture('image_pour_detection.jpg')

    
    

    img_draw = skimage.io.imread( 'image_pour_detection.jpg' )
    img = pre_process_image( img_draw )
    infer_image( graph, img )

    

    #traiter l'image avec object-detector
    #sorties
    
    #engine.say("I will speak this text")
    #engine.runAndWait()

camera.stop_preview()
engine.stop()
close_ncs_device( device, graph )

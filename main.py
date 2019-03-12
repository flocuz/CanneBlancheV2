from object-detector import *

from picamera import picamera
from time import sleep


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


close_ncs_device( device, graph )
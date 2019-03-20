# Text to speech
import pyttsx3
from subprocess import call

def getDirection( angle ) :
    angleMin = -1 * ANGLE_CAMERA * 0.5
    angleMax = ANGLE_CAMERA * 0.5
    res = ''
    if(angleMin <= angle and angle <= angleMin + (ANGLE_CAMERA/3)):
        res = 'Left'
    else:
        if (angleMin + (ANGLE_CAMERA / 3) <= angle and angle <= angleMax - (ANGLE_CAMERA / 3) ):
            res = 'Front'
        else:
            res = 'Right'
    return res
ANGLE_CAMERA = 62.2
print(getDirection(20.0))

print('Attention ' + 'lol' + ' ' + getDirection(20.0) + ' ' + str(1256.0/1000) + ' meters' )


#str = 'Look out ' + object[1].split(': ')[1] + ' ' + getDirection(angle) + ' ' + str(val/1000) + ' meters'
str = 'Look out ' + 'plane ' + ' ' + getDirection(20.0) + ' ' + str(2) + ' meters'
call(['espeak \'' + str + '\' 2>/dev/null'], shell=True)
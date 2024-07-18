import time
#import camera
from LineAvoidance import *
import threading

class LABD:
    while True:
    

        try:
            infrared.run()
            print("Before")
            #camera.picam2
            time.sleep(5)
            print("After")

        except KeyboardInterrupt:
            PWM.setMotorModel(0,0,0,0)
            break
        
print ("System stopped by user")     

# if __name__ =="__main__":
#     t1 = threading.Thread(target=LineAvoidance)
#     t2 = threading.Thread(target=camera)

#     t1.start()
#     t2.start()

#     t1.join()
#     t2.join()

#     print("Done!")

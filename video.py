#!/usr/bin/python 
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import socket
import io
import sys
import struct
from PIL import Image
from multiprocessing import Process
from Command import COMMAND as cmd
import yolov5
import os
import threading
import yolov5
import time


class VideoStreaming:
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier(r'haarcascade_frontalface_default.xml')
        self.video_Flag=True
        self.connect_Flag=False
        self.face_x=0
        self.face_y=0


        self.Wheel_Flag = 1
        self.Rotate_Flag = 1
        self.intervalChar = '#'
        self.endChar = '\n'

        self.target_color = None
        self.current_index = 0

        # Define color sequence (example)
        self.color_sequence = [0,1,2,3] #R,G,B,Y







        model_name = 'Yolov5_models'
        yolov5_model = 'balls5n.pt'
        CWD_PATH = os.getcwd()
        PATH_TO_YOLOV5_GRAPH = os.path.join(CWD_PATH, model_name, yolov5_model)

        self.model = yolov5.load(PATH_TO_YOLOV5_GRAPH)







    def StartTcpClient(self,IP):
        self.client_socket1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    def StopTcpcClient(self):
        try:
            self.client_socket.shutdown(2)
            self.client_socket1.shutdown(2)
            self.client_socket.close()
            self.client_socket1.close()
        except:
            pass

    def IsValidImage4Bytes(self,buf): 
        bValid = True
        if buf[6:10] in (b'JFIF', b'Exif'):     
            if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):
                bValid = False
        else:        
            try:  
                Image.open(io.BytesIO(buf)).verify() 
            except:  
                bValid = False
        return bValid
    
##########################NEW STUFF#########################
    def LedChange(self,R,G,B):
       

        #led_Off = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(0) + self.endChar
        color = self.intervalChar + str(R) + self.intervalChar + str(G) + self.intervalChar + str(B) + self.endChar

        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x01) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x02) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x04) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x08) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x10) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x20) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x40) + color)
        self.sendData(cmd.CMD_LED + self.intervalChar + str(0x80) + color)


 
    def face_detect(self,img):
        if sys.platform.startswith('win') or sys.platform.startswith('darwin'):
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            model_name = 'Yolov5_models'
            yolov5_model = 'balls5n.pt'
            model_labels = 'balls5n.txt'

            CWD_PATH = os.getcwd()
            PATH_TO_LABELS = os.path.join(CWD_PATH,model_name,model_labels)
            PATH_TO_YOLOV5_GRAPH = os.path.join(CWD_PATH,model_name,yolov5_model)

            # Import Labels File
            with open(PATH_TO_LABELS, 'r') as f:
                labels = [line.strip() for line in f.readlines()]
            # Initialize Yolov5
            model = yolov5.load(PATH_TO_YOLOV5_GRAPH)

            stride, names, pt = model.stride, model.names, model.pt
            print('stride = ',stride, 'names = ', names)
            #model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup

            min_conf_threshold = 0.5
            # set model parameters
            model.conf = 0.4  # NMS confidence threshold
            model.iou = 0.45  # NMS IoU threshold
            model.agnostic = False  # NMS class-agnostic
            model.multi_label = True # NMS multiple labels per box
            model.max_det = 5  # maximum number of detections per image

            frame = img.copy()
            results = model(frame)
            predictions = results.pred[0]

            boxes = predictions[:, :4]
            scores = predictions[:, 4]
            classes = predictions[:, 5]
            # Draws Bounding Box onto image
            results.render() 

            # Initialize frame rate calculation
            frame_rate_calc = 30
            freq = cv2.getTickFrequency()

            frame_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            #imW, imH = int(400), int(300)
            imW, imH = int(640), int(640)
            #frame_resized = cv2.resize(frame_rgb, (imW, imH))
            #input_data = np.expand_dims(frame_resized, axis=0)

            max_score = 0
            max_index = 0
            
            # Loop over all detections and draw detection box if confidence is above minimum threshold
            for i in range(len(scores)):
                curr_score = scores.numpy()
                # Found desired object with decent confidence
                if ((curr_score[i] > min_conf_threshold) and (curr_score[i] <= 1.0)):
                    print('Class: ',labels[int(classes[i])],' Conf: ', curr_score[i])

                    # Get bounding box coordinates and draw box
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    xmin = int(max(1,(boxes[i][0])))
                    ymin = int(max(1,(boxes[i][1])))
                    xmax = int(min(imW,(boxes[i][2])))
                    ymax = int(min(imH,(boxes[i][3])))
                            
                    # Draw label
                    #object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                    #label = '%s: %d%%' % (object_name, int(curr_score[i]*100)) # Example: 'person: 72%'
                    #labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                    #label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                    #cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
                    #cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                    #cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                    #if cType.getType() == "ball":

                    croppedImage = frame[ymin:ymax,xmin:xmax]
                    # Find Center
                    ccx = int((xmax - xmin)/2)
                    ccy = int((ymax - ymin)/2)

                    #pixel = croppedImage[ccx,ccy]
                    hsv_pixel = cv2.cvtColor(croppedImage, cv2.COLOR_BGR2HSV)
                    pixel_center = hsv_pixel[ccy,ccx]
                    hue_value = pixel_center[0]

                    ball_colornum = -1 #look at
                    
                    ball_color = 'No color detected'
                    #clear 
                    if hue_value in range(160, 180): #and dom1 in range(52,256) and dom2 in range(111, 255):
                      ball_colornum = 0
                      ball_color = ("RED")
                      self.LedChange(255,0,0)
                     
                    elif hue_value in range(70, 94): #and dom1 in range(52, 255) and dom2 in range(72, 255):
                      ball_colornum =1
                      ball_color =("GREEN") 
                      self.LedChange(0,255,0) 
                     
                    elif hue_value in range(95, 105): #and dom1 in range(80, 255) and dom2 in range(2, 255):
                      ball_colornum = 2
                      ball_color =("BLUE")
                      self.LedChange(0,0,255)
     
                    elif hue_value in range(24, 34):
                        ball_colornum = 3  
                        ball_color =("YELLOW")
                        self.LedChange(255,255,0)
       
                    print('Hue: ', hue_value, ball_color) 

                    # Record current max
                    max_score = curr_score[i]
                    max_index = i

                    if ball_colornum == self.color_sequence[self.current_index]:
                        self.move_forward()
                        time.sleep(2)
                        self.stop_movement()
                        self.current_index += 1
                        if self.current_index >= len(self.color_sequence):
                            self.color_sequence=0


            #put set to 0,0,0
            if len(scores) == 0:
                self.LedChange(0,0,0)
            cv2.imwrite("video.jpg",frame)

#################NEW STUFF####################

            results = self.model(img)
            predictions = results.pred[0]
            # Process predictions to extract ball coordinates
            ball_coords = []
            for pred in predictions:
                # Extract coordinates and other relevant information
                ball_coords.append(pred)
            return ball_coords            




    def move_forward(self, speed=1500):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_ForWard = self.intervalChar + str(0) + self.intervalChar + str(speed) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_ForWard)
            else:
                R_ForWard = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(speed) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_ForWard)
        else:
            ForWard = self.intervalChar + str(speed) + self.intervalChar + str(speed) + self.intervalChar + str(
                speed) + self.intervalChar + str(speed) + self.endChar
            self.sendData(cmd.CMD_MOTOR + ForWard)

    def move_backward(self, speed=1500):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_BackWard = self.intervalChar + str(180) + self.intervalChar + str(speed) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_BackWard)
            else:
                R_BackWard = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    180) + self.intervalChar + str(speed) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_BackWard)
        else:
            BackWard = self.intervalChar + str(-speed) + self.intervalChar + str(-speed) + self.intervalChar + str(
                -speed) + self.intervalChar + str(-speed) + self.endChar
            self.sendData(cmd.CMD_MOTOR + BackWard)

    def turn_left(self, angle=90, speed=1500):
        if self.Wheel_Flag:
            M_Turn_Left = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                angle) + self.intervalChar + str(speed) + self.endChar
            self.sendData(cmd.CMD_M_MOTOR + M_Turn_Left)
        else:
            Turn_Left = self.intervalChar + str(-speed) + self.intervalChar + str(-speed) + self.intervalChar + str(
                angle) + self.intervalChar + str(speed) + self.endChar
            self.sendData(cmd.CMD_MOTOR + Turn_Left)

    def turn_right(self, angle=90, speed=1500):
        if self.Wheel_Flag:
            M_Turn_Right = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                -angle) + self.intervalChar + str(speed) + self.endChar
            self.sendData(cmd.CMD_M_MOTOR + M_Turn_Right)
        else:
            Turn_Right = self.intervalChar + str(speed) + self.intervalChar + str(speed) + self.intervalChar + str(
                -angle) + self.intervalChar + str(-speed) + self.endChar
            self.sendData(cmd.CMD_MOTOR + Turn_Right)

    def stop_movement(self):
        if self.Wheel_Flag:
            if self.Rotate_Flag:
                M_Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_M_MOTOR + M_Stop)
            else:
                R_Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                    0) + self.intervalChar + str(0) + self.endChar
                self.sendData(cmd.CMD_CAR_ROTATE + R_Stop)
        else:
            Stop = self.intervalChar + str(0) + self.intervalChar + str(0) + self.intervalChar + str(
                0) + self.intervalChar + str(0) + self.endChar
            self.sendData(cmd.CMD_MOTOR + Stop)

    


    def streaming(self,ip):
        stream_bytes = b' '
        try:
            self.client_socket.connect((ip, 8000))
            self.connection = self.client_socket.makefile('rb')
        except:
            #print "command port connect failed"
            pass



###########################GPT###################################

        vs = cv2.VideoCapture(0)
        
        # def detect_balls():
        #     while True:
        #         ret, frame = vs.read()
        #         if not ret:
        #             break
                

        #     results = self.model(frame)
        #     predictions = results.pred[0]

        #     ball_coords = []
        #     for pred in predictions:
        #         # Extract coordinates and other relevant information
        #         ball_coords.append(pred)

        #     # Process ball coordinates as needed

        #     time.sleep(0.1)


        # def detect_faces():
        #     while True:
        #         ret, frame = vs.read()
        #         if not ret:
        #             break
        #         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #         faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        #     for (x, y, w, h) in faces:
        #         # Draw rectangle around detected faces if needed
        #         cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        #         time.sleep(0.1)
    
    # def main():
    #     ball_thread = threading.Thread(target=ball)
    #     boundary_thread = threading.Thread(target=boundary_detection)
    #     ball_thread.start()
    #     boundary_thread.start()
    #     ball_thread.join()
    #     boundary_thread.join()
    #     ball_thread = threading.Thread(target=detect_balls)
    #     boundary_thread = threading.Thread(target=detect_faces)

    #     ball_thread.start()
    #     boundary_thread.start()



#####################################################################
        while True:
            try:
                stream_bytes= self.connection.read(4) 
                leng=struct.unpack('<L', stream_bytes[:4])
                jpg=self.connection.read(leng[0])
                if self.IsValidImage4Bytes(jpg):
                            image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if self.video_Flag:
                                self.face_detect(image)
                                self.video_Flag=False

                                if self.target_color =='RED':
                                    # self.knockout()
                                    time.sleep(1)
                                    # self.find_next_ball()
                                
                            




            except Exception as e:
                print (e)
                break

        vs.release()
        self.client_socket.close()
        print("Streaming ended")    
                  
    def sendData(self,s):
        if self.connect_Flag:
            self.client_socket1.send(s.encode('utf-8'))

    def recvData(self):
        data=""
        try:
            data=self.client_socket1.recv(1024).decode('utf-8')
        except:
            pass
        return data

    def socket1_connect(self,ip):
        try:
            self.client_socket1.connect((ip, 5000))
            self.connect_Flag=True
            print ("Connection Successful !")
        except Exception as e:
            print ("Connect to server Failed!: Server IP is right? Server is opened?")
            self.connect_Flag=False

if __name__ == '__main__':
    pass

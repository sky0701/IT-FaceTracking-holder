import cv2
import dlib
from scipy.spatial import distance as dist
import imutils
from imutils.video import VideoStream
from imutils import face_utils
from imutils.video import FPS
from threading import Thread
import numpy as np
import playsound
#import playsound
import time
import math
#아두이노와 통신
import serial
import struct

arduino=serial.Serial('COM7',9600)
time.sleep(2)
print("done setup")

ALL = list(range(0, 68)) 
RIGHT_EYEBROW = list(range(17, 22))  
LEFT_EYEBROW = list(range(22, 27))  
RIGHT_EYE = list(range(36, 42))  
LEFT_EYE = list(range(42, 48))  
NOSE = list(range(27, 29))  
MOUTH_OUTLINE = list(range(48, 61))  
MOUTH_INNER = list(range(61, 68)) 
JAWLINE = list(range(0, 17)) 

index = ALL
check = 0
oldcheck = 0
def sound_alarm(path):
    # play an alarm sound
    playsound.playsound(path)

def eye_aspect_ratio(eye):
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    C = dist.euclidean(eye[0], eye[3])
 
    ear = (A + B) / (2.0 * C)
    return ear
def pro(img_frame,draw_rect1=True,draw_rect2=True,draw_lines=True):
    
    for face in detected_faces:
        global pitch
        global roll
        global yaw
        global check

        x1 = face.left()
        y1 =face.top()
        x2= face.right()
        y2= face.bottom()
        landmarks = face_pose_predictor(gray,face)

        size = img_frame.shape
        #2D image points. If you change the image, you need to change vector
        image_points = np.array([
                                (landmarks.part(33).x,landmarks.part(33).y),     # Nose tip
                                (landmarks.part(8).x,landmarks.part(8).y),       # Chin
                                (landmarks.part(36).x,landmarks.part(36).y),     # Left eye left corner
                                (landmarks.part(45).x,landmarks.part(45).y),     # Right eye right corne
                                (landmarks.part(48).x,landmarks.part(48).y),     # Left Mouth corner
                                (landmarks.part(54).x,landmarks.part(54).y)      # Right mouth corner
                            ], dtype="double")

        # 3D model points.
        model_points = np.array([
                                (0.0, 0.0, 0.0),             # Nose tip
                                (0.0, -330.0, -65.0),        # Chin
                                (-225.0, 170.0, -135.0),     # Left eye left corner
                                (225.0, 170.0, -135.0),      # Right eye right corne
                                (-150.0, -150.0, -125.0),    # Left Mouth corner
                                (150.0, -150.0, -125.0)      # Right mouth corner

                            ])
        # Camera internals
        focal_length = size[1]
        center = (size[1]/2, size[0]/2)
        camera_matrix = np.array(
                            [[focal_length, 0, center[0]],
                            [0, focal_length, center[1]],
                            [0, 0, 1]], dtype = "double"
                            )

        dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
        (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)

        axis = np.float32([[500,0,0],        [0,500,0],   [0,0,500]])
                          
        imgpts, jac = cv2.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        modelpts, jac2 = cv2.projectPoints(model_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        rvec_matrix = cv2.Rodrigues(rotation_vector)[0]

        proj_matrix = np.hstack((rvec_matrix, translation_vector))
        eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)[6] 

        
        pitch, yaw, roll = [math.radians(_) for _ in eulerAngles]


        pitch = math.degrees(math.asin(math.sin(pitch)))
        roll = -math.degrees(math.asin(math.sin(roll)))
        yaw = math.degrees(math.asin(math.sin(yaw)))
        check+=1

        #print(pitch,roll,yaw)
        cv2.putText(img_frame, 'pitch ' + str(int(pitch)),(30,70),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
        cv2.putText(img_frame, 'roll ' + str(int(roll)),(30,90),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
        cv2.putText(img_frame, 'yaw ' + str(int(yaw)),(30,110),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0),1,cv2.LINE_AA)
        
        (b1, jacobian) = cv2.projectPoints(np.array([(350.0, 270.0, 0.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b2, jacobian) = cv2.projectPoints(np.array([(-350.0, -270.0, 0.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b3, jacobian) = cv2.projectPoints(np.array([(-350.0, 270, 0.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b4, jacobian) = cv2.projectPoints(np.array([(350.0, -270.0, 0.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)

        (b11, jacobian) = cv2.projectPoints(np.array([(450.0, 350.0, 400.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b12, jacobian) = cv2.projectPoints(np.array([(-450.0, -350.0, 400.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b13, jacobian) = cv2.projectPoints(np.array([(-450.0, 350, 400.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
        (b14, jacobian) = cv2.projectPoints(np.array([(450.0, -350.0, 400.0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)

        b1 = ( int(b1[0][0][0]), int(b1[0][0][1]))
        b2 = ( int(b2[0][0][0]), int(b2[0][0][1]))
        b3 = ( int(b3[0][0][0]), int(b3[0][0][1]))
        b4 = ( int(b4[0][0][0]), int(b4[0][0][1]))

        b11 = ( int(b11[0][0][0]), int(b11[0][0][1]))
        b12 = ( int(b12[0][0][0]), int(b12[0][0][1]))
        b13 = ( int(b13[0][0][0]), int(b13[0][0][1]))
        b14 = ( int(b14[0][0][0]), int(b14[0][0][1]))

        if draw_rect1 ==True:
            cv2.line(img_frame,b1,b3,(0,0,0),5)
            cv2.line(img_frame,b3,b2,(0,0,0),5)
            cv2.line(img_frame,b2,b4,(0,0,0),5)
            cv2.line(img_frame,b4,b1,(0,0,0),5)

        if draw_rect2 ==True:
            cv2.line(img_frame,b11,b13,(0,0,0),5)
            cv2.line(img_frame,b13,b12,(0,0,0),5)
            cv2.line(img_frame,b12,b14,(0,0,0),5)
            cv2.line(img_frame,b14,b11,(0,0,0),5)

        if draw_lines == True:
            cv2.line(img_frame,b11,b1,(0,0,0),5)
            cv2.line(img_frame,b13,b3,(0,0,0),5)
            cv2.line(img_frame,b12,b2,(0,0,0),5)
            cv2.line(img_frame,b14,b4,(0,0,0),5)



    #return img_frame


EYE_AR_THRESH = 0.25

EYE_AR_CONSEC_FRAMES = 35

alarm_path= "alarm.wav"

COUNTER = 0

ALARM_ON = False

face_detector = dlib.get_frontal_face_detector()

face_pose_predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks.dat')

(lStart, lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
(rStart, rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

vs = VideoStream(src=0).start()
time.sleep(1.0)

fps = FPS().start()
fps
framePerSecond = 30.0
time = cv2.getTickCount()
count=0
loopcount=0
framesSkipping=2
P=R=Y=DX=DY=0
width=400
sleep_detect=1 #안자는 상태
init_start=0
P_init=R_init=Y_init=DX_init=DY_init=0
sit=0
while True:
    if count==0:
        time=cv2.getTickCount()
    fps.update()
    frame = vs.read()
    frame = imutils.resize(frame, width)
    if sit==1:
        frame=cv2.rotate(frame,cv2.ROTATE_180) # 앉아서 사용할 때 카메라 180도 돌림
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    detected_faces = face_detector(gray, 0)
    fps.stop()
    oldcheck = check + 1
    pro(frame)
    fpscor=(frame.shape[1]-110,20)
    cv2.putText(frame, "FPS: {:.2f}".format(fps.fps()), fpscor, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,30,20), 2)
    for face in detected_faces:
        shape = face_pose_predictor(frame, face) #얼굴에서 68개 점 찾기
        list_points = []
        for p in shape.parts():
            list_points.append([p.x, p.y])
        list_points = np.array(list_points)
        for i,pt in enumerate(list_points[index]):

            pt_pos = (pt[0], pt[1])
            cv2.putText(frame,str(i),(pt[0],pt[1]),cv2.FONT_HERSHEY_COMPLEX,0.3,(0,225,0)) 
        #코의 위치-> 이것을 dx dy 값으로 쓴다.
        cv2.putText(frame,str(int(list_points[30][0]-width/2))+str(int(list_points[30][1]-width/2*2/3)),(30,50),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,0)) 
        cv2.putText(frame,'eye distance: '+str(list_points[42][0]-list_points[39][0]),(250,250),cv2.FONT_HERSHEY_COMPLEX,0.5,(250,250,0)) 
        cv2.putText(frame,'Distance: '+str(1360/(list_points[42][0]-list_points[39][0])),(250,230),cv2.FONT_HERSHEY_COMPLEX,0.5,(250,250,0)) 
        dx=list_points[30][0]-width/2
        dy=(list_points[30][1]-width/2*2/3)*(-1) #상하 반전 보정
    
    if check!=0:
        if check==oldcheck:
            P=P+int(pitch)
            R=R+int(roll)
            Y=Y+int(yaw)
            DX=DX+int(dx)
            DY=DY+int(dy)
            if check%4==0:
                P=int(P/4)
                R=int(R/4)
                Y=int(Y/4)
                DX=int(DX/4)
                DY=int(DY/4)
                if init_start==1:
                    print('초기화 완료')
                    P_init=P    
                    R_init=R
                    Y_init=Y
                    DX_init=DX
                    DY_init=DY

                print("P:",P-P_init,"R:",R-R_init,"Y:",Y-Y_init,"DX:",DX-DX_init,"DY:",DY-DY_init,"자니?:",sleep_detect)
                #바이너리 0<=number<=255만 표현 가능, 100을 임의로 더해서 전송하고, 아두이노에서 빼자
                if 0<=(P-P_init+500)/10<=255 and 0<=(R-R_init+500)/10<=255 and 0<=(Y-Y_init+500)/10<=255 and 0<=(DX-DX_init+500)/10<=255 and 0<=(DY-DY_init+500)/10<=255:
                    arduino.write(struct.pack('>BBBBBB',(P-P_init+500)//10,(R-R_init+500)//10,(Y-Y_init+500)//10,(DX-DX_init+500)//10,(DY-DY_init+500)//10,sleep_detect))
                else:
                    print('skip')
                P=R=Y=DX=DY=init_start=0
        else: 
            P=R=Y=DX=DY=check=oldcheck=0
    

    for rect in detected_faces:
        detected_faces = face_pose_predictor(gray, rect)
        detected_faces = face_utils.shape_to_np(detected_faces)

        leftEye = detected_faces[lStart:lEnd]
        rightEye = detected_faces[rStart:rEnd]
        leftEAR = eye_aspect_ratio(leftEye)        
        rightEAR = eye_aspect_ratio(rightEye)
        ear = (leftEAR + rightEAR) / 2.0

        if ear < EYE_AR_THRESH:
            COUNTER += 1                
            if COUNTER >= EYE_AR_CONSEC_FRAMES:
                if not ALARM_ON:
                    ALARM_ON = True
                    t = Thread(target=sound_alarm,args=(alarm_path,))
                    t.deamon = True
                    t.start()
                

                alertcor=(int(frame.shape[1]/2-180),70)
                sleep_detect=2 #잔다
                cv2.putText(frame, "TRIGGER DROWSINESS ALARM!", alertcor, cv2.FONT_HERSHEY_SIMPLEX , 0.5, (0, 0, 255), 2)

        else:
            COUNTER = 0
            sleep_detect=1 #안잔다
            ALARM_ON = False
       
        EARcor=(int(frame.shape[1]/2-180),200)
        cv2.putText(frame, "Eye Aspect Ratio: {:.2f}".format(ear), EARcor,cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, ), 1)
 
    cv2.imshow("Detector", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    elif key ==ord("c"):
        print('보정 시작합니다. 편한자세로 스크린의 정면을 바라봐주세요')
        P=R=Y=DX=DY=check=oldcheck=0
        P_init=R_init=Y_init=DX_init=DY_init=0
        init_start=1
    elif key==ord("s"):
        if sit==0:
            sit=1
        else:
            sit=0
    


cv2.destroyAllWindows()
vs.stop()
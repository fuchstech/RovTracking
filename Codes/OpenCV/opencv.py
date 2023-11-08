
import cv2
print(cv2.__version__)
import numpy as np
 
def nothing(x):
    pass
 
cv2.namedWindow('Trackbars')
cv2.moveWindow('Trackbars',1320,0)
 
cv2.createTrackbar('hueLower', 'Trackbars',18,179,nothing)
cv2.createTrackbar('hueUpper', 'Trackbars',56,179,nothing)
 
cv2.createTrackbar('hue2Lower', 'Trackbars',179,179,nothing)
cv2.createTrackbar('hue2Upper', 'Trackbars',179,179,nothing)
 
cv2.createTrackbar('satLow', 'Trackbars',217,255,nothing)
cv2.createTrackbar('satHigh', 'Trackbars',255,255,nothing)
cv2.createTrackbar('valLow','Trackbars',40,255,nothing)
cv2.createTrackbar('valHigh','Trackbars',255,255,nothing)
 
 
dispW=640
dispH=480
flip=2
#Uncomment These next Two Line for Pi Camera
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam= cv2.VideoCapture("video.mp4")
 
#Or, if you have a WEB cam, uncomment the next line
#(If it does not work, try setting to '1' instead of '0')
#cam=cv2.VideoCapture(0)
while True:
    #frame = cv2.imread("video.png")
    ret, frame= cam.read()
    #cv2.resize(frame, (100,100))
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
 
    hueLow=cv2.getTrackbarPos('hueLower', 'Trackbars')
    hueUp=cv2.getTrackbarPos('hueUpper', 'Trackbars')
 
    hue2Low=cv2.getTrackbarPos('hue2Lower', 'Trackbars')
    hue2Up=cv2.getTrackbarPos('hue2Upper', 'Trackbars')
 
    Ls=cv2.getTrackbarPos('satLow', 'Trackbars')
    Us=cv2.getTrackbarPos('satHigh', 'Trackbars')
 
    Lv=cv2.getTrackbarPos('valLow', 'Trackbars')
    Uv=cv2.getTrackbarPos('valHigh', 'Trackbars')
 
    l_b=np.array([hueLow,Ls,Lv])
    u_b=np.array([hueUp,Us,Uv])
 
    l_b2=np.array([hue2Low,Ls,Lv])
    u_b2=np.array([hue2Up,Us,Uv])
 
    FGmask=cv2.inRange(hsv,l_b,u_b)
    FGmask2=cv2.inRange(hsv,l_b2,u_b2)
    FGmaskComp=cv2.add(FGmask,FGmask2)
    cv2.imshow('FGmaskComp',FGmaskComp)
    cv2.moveWindow('FGmaskComp',0,530)
 
    contours,_=cv2.findContours(FGmaskComp,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contours=sorted(contours,key=lambda x:cv2.contourArea(x),reverse=True)
    for cnt in contours:
        area=cv2.contourArea(cnt)
        (x,y,w,h)=cv2.boundingRect(cnt)
        if area>=7000:
            #cv2.drawContours(frame,[cnt],0,(255,0,0),3)
            #cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),3)
            cv2.line(frame,(x,int(y+h/2)),(x+w,int(y+h/2)),(255,0,0),3)
            cv2.line(frame,(int(x+w/2), y), (int(x+w/2), y+h), (255,2,2), 3)
    cv2.imshow('nanoCam',frame)
    cv2.moveWindow('nanoCam',0,0)
    
 
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
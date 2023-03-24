import numpy as np
import cv2
import Person
import time
import paho.mqtt.client as mqtt
import json
import redis
import argparse

def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', help='Hostname', nargs='+', required=True)
    parser.add_argument('--w', help='width', required=True)
    parser.add_argument('--h', help='height', required=True)
    parser.add_argument('--track', help='Tracking', required=True)
    parser.add_argument('--lup', help='Line Up', type=float, required=True)
    parser.add_argument('--ldown', help='Line Down', type=float, required=True)
    parser.add_argument('--elimit', help='External limit', type=float, required=True)
    parser.add_argument(
        '--password', help='Redis password', default="redis password")
    args = parser.parse_args()
    return args

args = _parse_args()

device = 'device name' #used on MQTT msg

j = int(len(args.host))
r = []

for i in range(j):
    r.append(redis.Redis(host=str(args.host[i]), port=6379, password="your_pass"))

# Define o tamanho da imagem
width = int(args.w)
height = int(args.h)


payload = ""

broker = "your broker"
broker_port = 1883
timeout_reconnect = 60

wait_time = 5

client = mqtt.Client('client')
client.connect(broker, broker_port, timeout_reconnect)
client.loop_start()

def on_message(client, userdata, msg):
  global payload
  payload = json.loads(msg.payload)

client.on_message = on_message

cnt_up   = 0
cnt_down = 0

w = width
h = height

w_ = (w * j)


frameArea = h*w_

div = 1250 #250

areaTH = frameArea/div
print('Area Threshold', areaTH)


lu = float(args.lup)
ld = float(args.ldown)
ed = float(args.elimit)

line_up = int(lu*(h/5))
line_down   = int(ld*(h/5))

up_limit =   int((lu - ed)*(h/5))
down_limit = int((ld + ed)*(h/5))

print("Red line y:",str(line_down))
print("Blue line y:", str(line_up))
line_down_color = (255,0,0)
line_up_color = (0,0,255)
pt1 =  [0, line_down];
pt2 =  [w_, line_down];
pts_L1 = np.array([pt1,pt2], np.int32)
pts_L1 = pts_L1.reshape((-1,1,2))
pt3 =  [0, line_up];
pt4 =  [w_, line_up];
pts_L2 = np.array([pt3,pt4], np.int32)
pts_L2 = pts_L2.reshape((-1,1,2))

pt5 =  [0, up_limit];
pt6 =  [w_, up_limit];
pts_L3 = np.array([pt5,pt6], np.int32)
pts_L3 = pts_L3.reshape((-1,1,2))
pt7 =  [0, down_limit];
pt8 =  [w_, down_limit];
pts_L4 = np.array([pt7,pt8], np.int32)
pts_L4 = pts_L4.reshape((-1,1,2))

#Substractor de fondo
fgbg = cv2.createBackgroundSubtractorMOG2(detectShadows = True)

#Elementos estructurantes para filtros morfoogicos
kernelOp = np.ones((3,3),np.uint8)
kernelOp2 = np.ones((5,5),np.uint8)
kernelCl = np.ones((11,11),np.uint8)

#Variables
font = cv2.FONT_HERSHEY_SIMPLEX
persons = []
max_p_age = 5
pid = 1


def getserial():
  # Extract serial from cpuinfo file
  # just Raspberry
  cpuserial = "0000000000000000"
  try:
    f = open('/proc/cpuinfo','r')
    for line in f:
      if line[0:6]=='Serial':
        cpuserial = line[10:26]
    f.close()
  except:
    cpuserial = "ERROR000000000"
  return cpuserial

device_id = device

while True:
    jpg_as_text = []
    jpg_buffer = []
    frame_ = []
    f = []

    for i_ in range(j):
        jpg_as_text.append(r[int(i_)].get('video_feed'))
        jpg_buffer.append(np.frombuffer(jpg_as_text[i_], dtype=np.uint8))
        frame_.append(cv2.imdecode(jpg_buffer[i_], cv2.IMREAD_COLOR))
        frame_[i_] = cv2.resize(frame_[i_], (width, height))
        f.append(frame_[i_])

    result_frame = cv2.hconcat(f)
 

    frame = result_frame

    for i in persons:
        i.age_one() 
    
    fgmask = fgbg.apply(frame)
    fgmask2 = fgbg.apply(frame)

    try:
        ret,imBin= cv2.threshold(fgmask,200,255,cv2.THRESH_BINARY)
        ret,imBin2 = cv2.threshold(fgmask2,200,255,cv2.THRESH_BINARY)
        #Opening (erode->dilate) para quitar ruido.
        mask = cv2.morphologyEx(imBin, cv2.MORPH_OPEN, kernelOp)
        mask2 = cv2.morphologyEx(imBin2, cv2.MORPH_OPEN, kernelOp)
        #Closing (dilate -> erode) para juntar regiones blancas.
        mask =  cv2.morphologyEx(mask , cv2.MORPH_CLOSE, kernelCl)
        mask2 = cv2.morphologyEx(mask2, cv2.MORPH_CLOSE, kernelCl)
    except:
        print('EOF')
        print('UP:',cnt_up)
        print('DOWN:',cnt_down)
        break

    contours0, hierarchy = cv2.findContours(mask2,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours0:
        area = cv2.contourArea(cnt)
        if area > areaTH:
 
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            x,y,w,h = cv2.boundingRect(cnt)

            new = True

            if cy in range(up_limit,down_limit):
                for i in persons:
                    if abs(cx-i.getX()) <= w and abs(cy-i.getY()) <= h:
                        new = False
                        i.updateCoords(cx,cy)   
                        if i.going_UP(line_down,line_up) == True:
                            cnt_up += 1;
                            print("ID:",i.getId(),'crossed going up at',time.strftime("%c"))
                            broker_out  = {"event":"people_count", "device_id": str(device_id), "session_id":str(i.getId()), "status":"up"}
                            data_out  = json.dumps(broker_out)
                            client.publish("pcount", data_out, qos=2)
                        elif i.going_DOWN(line_down,line_up) == True:
                            cnt_down += 1;
                            print("ID:",i.getId(),'crossed going down at',time.strftime("%c"))
                            broker_out  = {"event":"people_count", "device_id": str(device_id), "session_id":str(i.getId()), "status":"down"}
                            data_out  = json.dumps(broker_out)
                            client.publish("pcount", data_out, qos=2)
                        break
                    if i.getState() == '1':
                        if i.getDir() == 'down' and i.getY() > down_limit:
                            i.setDone()
                        elif i.getDir() == 'up' and i.getY() < up_limit:
                            i.setDone()
                    if i.timedOut():
                        #sacar i de la lista persons
                        index = persons.index(i)
                        persons.pop(index)
                        del i     #liberar la memoria de i
                if new == True:
                    p = Person.MyPerson(pid,cx,cy, max_p_age)
                    persons.append(p)
                    pid += 1

            cv2.circle(frame,(cx,cy), 5, (0,0,255), -1)
            #img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            img = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            #cv2.drawContours(frame, cnt, -1, (0,255,0), 3)


    if(args.track == "True"):
        for i in persons:
            if len(i.getTracks()) >= 2:
                pts = np.array(i.getTracks(), np.int32)
                pts = pts.reshape((-1,1,2))
                frame = cv2.polylines(frame,[pts],False,i.getRGB())
            if i.getId() == 9:
                print(str(i.getX()), ',', str(i.getY()))
            cv2.putText(frame, str(i.getId()),(i.getX(),i.getY()),font,0.3,i.getRGB(),1,cv2.LINE_AA)


    str_up = 'UP: '+ str(cnt_up)
    str_down = 'DOWN: '+ str(cnt_down)
    frame = cv2.polylines(frame,[pts_L1],False,line_down_color,thickness=2)
    frame = cv2.polylines(frame,[pts_L2],False,line_up_color,thickness=2)
    frame = cv2.polylines(frame,[pts_L3],False,(255,255,255),thickness=1)
    frame = cv2.polylines(frame,[pts_L4],False,(255,255,255),thickness=1)
    cv2.putText(frame, str_up ,(10,40),font,0.5,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame, str_up ,(10,40),font,0.5,(0,0,255),1,cv2.LINE_AA)
    cv2.putText(frame, str_down ,(10,90),font,0.5,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame, str_down ,(10,90),font,0.5,(255,0,0),1,cv2.LINE_AA)

    cv2.imshow('Frame',frame)
    #cv2.imshow('Mask',mask)

    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

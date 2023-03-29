import cv2
import redis

r = redis.Redis(host='localhost', port=6379, password="your pass")

cap = cv2.VideoCapture(0)

width = 640
height = 480

while(True):
    ret, frame = cap.read()

    frame = cv2.resize(frame, (width, height))

    encoded, buffer = cv2.imencode('.jpg', frame)

    jpg_as_text = buffer.tobytes()

    r.set('video_feed', jpg_as_text)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

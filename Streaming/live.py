import cv2
import numpy as np
import redis

# Initialize the redis connection
# Need to install redis-serv on source device
# after the install, define bind 0.0.0.0 and deefine too your password on requirepass line
r = redis.Redis(host='your_host', port=6379, password="your_pass")

# Image size
width = 640
height = 480

while True:
    jpg_as_text = r.get('video_feed')

    jpg_buffer = np.frombuffer(jpg_as_text, dtype=np.uint8)

    frame = cv2.imdecode(jpg_buffer, cv2.IMREAD_COLOR)

    frame = cv2.resize(frame, (width, height))

    # optional
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
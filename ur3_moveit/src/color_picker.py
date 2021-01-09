import numpy as np
import cv2

cap = cv2.VideoCapture(6) # if fails -> check if camera stream is not already used somewhere
cap.set(3,424)
cap.set(4,240)

window_name = "HSV Color Picker"

cv2.namedWindow(window_name)
 
def empty_callback(x):
    pass

cv2.createTrackbar("L - H", window_name, 0, 179, empty_callback)
cv2.createTrackbar("U - H", window_name, 179, 179, empty_callback)
cv2.createTrackbar("L - S", window_name, 0, 255, empty_callback)
cv2.createTrackbar("U - S", window_name, 255, 255, empty_callback)
cv2.createTrackbar("L - V", window_name, 0, 255, empty_callback)
cv2.createTrackbar("U - V", window_name, 255, 255, empty_callback)


while True:
    
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.flip( frame, 1 ) 
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    low_h = cv2.getTrackbarPos("L - H", window_name)
    low_s = cv2.getTrackbarPos("L - S", window_name)
    low_v = cv2.getTrackbarPos("L - V", window_name)
    upper_h = cv2.getTrackbarPos("U - H", window_name)
    upper_s = cv2.getTrackbarPos("U - S", window_name)
    upper_v = cv2.getTrackbarPos("U - V", window_name)
 
    lower_range = np.array([low_h, low_s, low_v])
    upper_range = np.array([upper_h, upper_s, upper_v])
    
    mask = cv2.inRange(hsv, lower_range, upper_range)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    mask_gs = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    stacked = np.hstack((mask_gs,frame,res))
    
    scale = 1.0
    cv2.imshow(window_name,cv2.resize(stacked,None,fx=scale,fy=scale)) # TODO add slider for image size
    
    key = cv2.waitKey(1)
    if key == 27: # ESC to quit
        break
    
cap.release()
cv2.destroyAllWindows()
import cv2
import glob

imagelist = glob.glob("*.pgm")
imagelist.sort()

def diffImg(t0, t1, t2):
  d1 = cv2.absdiff(t2, t1)
  d2 = cv2.absdiff(t1, t0)
  return cv2.bitwise_and(d1, d2)
 
winName = "Movement Indicator"
cv2.namedWindow(winName, cv2.CV_WINDOW_AUTOSIZE)
 
# Read three images first:
t_minus = cv2.imread(imagelist[1])
t = cv2.imread(imagelist[2])
t_plus = cv2.imread(imagelist[3])

with open("movement.log", 'w') as mvf:
 
    for img in imagelist[3:]:
        with open(img[:-4] + ".time", 'r') as f:
            time = f.readline()
            time = time.rstrip('\r\n')
    
        diffimage = cv2.absdiff(t_minus, t)
        cv2.imshow( winName, diffimage )
        mean = cv2.mean(diffimage)
 
        mvf.write(time + "\t" + str(mean[1]*10) + "\n")
    
        # Read next image
        t_minus = t
        t = t_plus
        t_plus = cv2.imread(img)
    
        key = cv2.waitKey(10)
        if key == 27:
            cv2.destroyWindow(winName)
            break
 
print "Goodbye"

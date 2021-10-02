import cv2
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


img = mpimg.imread('uvalda_05.png')
print(img.shape)
print(img)
o = cv2.distanceTransform(np.uint8(img), 2, 5)
plt.imshow(o)
plt.colorbar()
plt.show()
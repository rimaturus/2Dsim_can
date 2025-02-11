import cv2
import numpy as np
import matplotlib.pyplot as plt

# 1) Read the screenshot image
image = cv2.imread("Screenshots_1.png")

# 2) Convert to HSV
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 3) Define a range for "gray" in HSV (tweak if needed)
lower_gray = np.array([0, 0, 120], dtype=np.uint8)
upper_gray = np.array([180, 40, 255], dtype=np.uint8)

# 4) Create a binary mask of the track
mask = cv2.inRange(hsv, lower_gray, upper_gray)

# 5) Morphological "close" to fill any thin gaps/lines
#    Increase the kernel size if the crossing lines are thick
kernel = np.ones((25,25), np.uint8)
filled_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# 6) Find contours on the *filled* mask
#    - RETR_EXTERNAL to get only the outer boundary
#    - use RETR_CCOMP/RETR_TREE if you also need inner holes
contours, hierarchy = cv2.findContours(
    filled_mask,
    cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE
)

# 7) Draw those “cleaned up” contours
contour_img = np.zeros_like(image)              # black BGR image
cv2.drawContours(contour_img, contours, -1, (0, 0, 255), 2)  # red

# 8) Show the results side by side
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))

ax1.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
ax1.set_title("Original Image")
ax1.axis("off")

ax2.imshow(mask, cmap='gray')
ax2.set_title("Initial Mask (with extra lines)")
ax2.axis("off")

ax3.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
ax3.set_title("Cleaned-Up Track Border")
ax3.axis("off")

plt.tight_layout()
plt.show()

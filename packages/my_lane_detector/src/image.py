#!/usr/bin/env python3

import cv2
import numpy as np

# Load the saved image
image_path = "image.png"  # Replace with your actual file name
img = cv2.imread(image_path)
assert img is not None, "Image not found. Check the file name and path."

# Convert to grayscale for Canny
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

### Step 1: Canny Edge Detection ###
canny_thresholds = [(50, 150), (100, 200), (150, 250)]
for low, high in canny_thresholds:
    edges = cv2.Canny(gray, low, high)
    cv2.imwrite(f"canny_{low}_{high}.png", edges)

### Step 2: Hough Transform with varying thresholds ###
edges_for_hough = cv2.Canny(gray, 100, 200)  # Pick one edge map
hough_thresholds = [30, 50, 70]
for h_thresh in hough_thresholds:
    output = img.copy()
    lines = cv2.HoughLinesP(edges_for_hough, 1, np.pi / 180, threshold=h_thresh,
                            minLineLength=50, maxLineGap=10)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(output, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv2.imwrite(f"hough_thresh_{h_thresh}.png", output)

### Step 3: HSV vs RGB Yellow Detection ###
# HSV yellow mask
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
yellow_hsv = cv2.inRange(hsv, (15, 80, 80), (30, 255, 255))
yellow_hsv_result = cv2.bitwise_and(img, img, mask=yellow_hsv)
cv2.imwrite("yellow_hsv.png", yellow_hsv_result)

# RGB yellow mask
lower_yellow_rgb = np.array([200, 200, 0])
upper_yellow_rgb = np.array([255, 255, 100])
yellow_rgb = cv2.inRange(img, lower_yellow_rgb, upper_yellow_rgb)
yellow_rgb_result = cv2.bitwise_and(img, img, mask=yellow_rgb)
cv2.imwrite("yellow_rgb.png", yellow_rgb_result)

### Step 4: Lighting Condition Test ###
# Simulate darker and brighter images
dark_img = cv2.convertScaleAbs(img, alpha=0.5, beta=0)
bright_img = cv2.convertScaleAbs(img, alpha=1.5, beta=30)

# Use same HSV yellow detection
for variant_name, variant in [("dark", dark_img), ("bright", bright_img)]:
    hsv_variant = cv2.cvtColor(variant, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_variant, (15, 80, 80), (30, 255, 255))
    result = cv2.bitwise_and(variant, variant, mask=yellow_mask)
    cv2.imwrite(f"lighting_{variant_name}.png", result)

import cv2
import numpy as np
import time

## Ouput video preparation
codec = cv2.VideoWriter_fourcc(*'mp4v')
output = cv2.VideoWriter('output.mp4', codec, 20.0, (640, 480))

# Initialize andriod webcam using IP webcam
cap = cv2.VideoCapture(0)

# Allow the system to sleep for 3 seconds before webcam starts
time.sleep(3)

# Take the background image first
for i in range(10):
    success, background = cap.read()
background = np.flip(background, axis=1)

while (cap.isOpened()):
    success, frame = cap.read()
    if not success:
        break

    frame = np.flip(frame, axis=1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the color range for the cloak and create a mask for the specified color range
    lower_red = np.array([0,120,70])
    upper_red = np.array([10,255,255])
    mask_1 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170,120,70])
    upper_red = np.array([180,255,255])
    mask_2 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask_1 + mask_2

    # Perform morphological operations to clean up the mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, np.ones((3,3), np.uint8))

    # Invert the mask to get the inverse mask
    inverse_mask = cv2.bitwise_not(mask)

    # Extract the region of interest from the background using mask
    cloak_area = cv2.bitwise_and(background, background, mask=mask)

    # Capture the background without the cloak
    background_area = cv2.bitwise_and(frame, frame, mask=inverse_mask)

    # Combine the cloak area and background to get the final output and resize the final output to video resolution
    final_image = cv2.add(cloak_area, background_area)
    final_image = cv2.resize(final_image ,(640,480))

    # Display the output
    cv2.imshow("Invisibility Cloak", final_image)

    # Save the frame to the output video (optional)
    output.write(final_image)

    # Break the loop when 'esc' key is pressed
    if cv2.waitKey(1) == 27:
        break

# Release the webcam and close all windows
cap.release()
output.release()
cv2.destroyAllWindows()

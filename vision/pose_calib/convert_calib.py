import cv2

c = cv2.FileStorage("calib_3.yml", cv2.FILE_STORAGE_READ)

c2 = cv2.FileStorage("calib_3.json", cv2.FILE_STORAGE_WRITE)

#c2.write("calibration_date", c.getNode("calibration_time"))
#c2.write("camera_resolution", c.getNode("camera_resolution"))
c2.write("camera_matrix", c.getNode("camera_matrix").mat())
c2.write("distortion_coefficients", c.getNode("distortion_coefficients").mat())
c2.release()

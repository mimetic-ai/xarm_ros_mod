#!/usr/bin/env python3
import rospy # Python library for ROS
import cv2 # OpenCV library
# import openvino.runtime as ov
from sensor_msgs.msg import Image, CompressedImage, CameraInfo # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import imutils

PX_SIZE = 2.9e-6
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGIAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
def roll(x):
   return np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])


def yaw(x):
    return np.array([[np.cos(x), -np.sin(x), 0], [np.sin(x), np.cos(x), 0], [0, 0, 1]])


def pitch(x):
    return np.array([[np.cos(x), 0, np.sin(x)], [0, 1, 0], [-np.sin(x), 0, np.cos(x)]])


R1 = roll(-np.pi/2)@yaw(-np.pi/2)@pitch(45*np.pi/180)@yaw((180-15)*np.pi/180)
R2 = roll(-np.pi/2)@yaw(-np.pi/2)@pitch(45*np.pi/180)@yaw((180+7)*np.pi/180)

t1 = np.array([[0.02, -0.26, 0.27]]).T
t2 = np.array([[0.02, 0.27, 0.27]]).T

M1 = np.concatenate((R1, -R1@t1), axis=1)
# M1 = np.concatenate((R1, -t1), axis=1)
M2 = np.concatenate((R2, -R2@t2), axis=1)
# M2 = np.concatenate((R2, -t2), axis=1)

# camera_matrix = np.array([[1.49684218e+03, 0.00000000e+00, 8.88271095e+02],
#  [0.00000000e+00, 1.48468507e+03, 4.28626260e+02],
#  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

cam_mat1 = np.array([[4.32863623e-03, 0.00000000e+00, 3.00038817e-03],
 [0.00000000e+00, 4.32919521e-03, 1.43377857e-03],
 [0.00000000e+00, 0.00000000e+00, 1]]
)

cam_mat2 = np.array( [[4.41363824e-03, 0.00000000e+00, 2.89790186e-03],
 [0.00000000e+00, 4.41046262e-03, 1.43522482e-03],
 [0.00000000e+00, 0.00000000e+00, 2.90000000e-06]]
)

# camera_matrix[:-1, :] *= PX_SIZE # Puts intrinsic mat into meters
proj1 = cam_mat1 @ M1
proj2 = cam_mat2 @ M2
proj1 = np.array([[589.3664541825391, 0.0, 320.5], [-41.25565179277774, 0.0, 589.3664541825391], [240.5, 0.0, 0.0], [0.0, 1.0, 0.0]])
proj2 = np.array([[589.3664541825391, 0.0, 320.5], [-41.25565179277774, 0.0, 589.3664541825391], [240.5, 0.0, 0.0], [0.0, 1.0, 0.0]])
proj1 = np.reshape(proj1, (3, 4))
proj2 = np.reshape(proj2, (3, 4))
# proj1 = np.array([[ 4.15661781e+02,  1.11376239e+02,  1.68652977e+03,  2.08497852e+02], [-6.77021952e+02,  1.35565160e+03,  3.03084535e+02,  3.34145588e+02],[-6.83012702e-01, -1.83012702e-01,  7.07106781e-01,  2.60000000e-01]], dtype=np.float32)

# proj2 = np.array([[ 3.72672158e+02, -2.15162371e+02,  1.68652977e+03, 2.08497852e+02],
#  [ 4.79863628e+02,  1.43731725e+03,  3.03084535e+02,  3.34145588e+02],
#  [-6.12372436e-01,  3.53553391e-01,  7.07106781e-01,  2.60000000e-01]], dtype=np.float32)

# camera_matrix = np.array([[1.49684218e+03, 0.00000000e+00, 8.88271095e+02], [0.00000000e+00, 1.48468507e+03, 4.28626260e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
# camera_matrix[:-1, :] *= PX_SIZE
dist_coeffs =  np.array([[ 0.08179988, -0.077251, -0.00262296,  0.00596973, -0.05618906]])
corners_list = [None, None]
ids_list = [None, None]

def centerPoint3d(corners1, corners2):
  # print(type(proj1), proj1.dtype)
  corners1 = cv2.undistortPoints(corners1, cam_mat1, dist_coeffs)
  corners2 = cv2.undistortPoints(corners2, cam_mat2, dist_coeffs)
  print('\nundistorted_corners1:\n', corners1)
  print('\nundistorted_corners2:\n', corners2)
  points3d = cv2.triangulatePoints(projMatr1 = proj1, projMatr2 = proj2, projPoints1 = corners1, projPoints2 = corners2)
  print("\nraw_points3d:\n", points3d)
  last_row = np.array([points3d[-1,:]]).T
  #last_col = np.array([points3d[:, -1]])
  points3d = points3d/last_row
  print("\nnormalized_points_3d:\n", points3d, "\n")
  points3d_3 = points3d[:3,:]
  # # print(points3d_3)
  return np.average(points3d_3, axis = 1)
  # return (sum/4)




def callback(data, args):

  i = args[0]
  arucoDict = args[1]
  arucoParams = args[2]
  print("entered callback")

 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  # rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  current_frame = br.compressed_imgmsg_to_cv2(data)
  # print("corners list printing")
  # print(corners_list)
  # print("ids list printing")
  # print(ids_list)
  # image = imutils.resize(current_frame, width=600)
  if i == 0:
    (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)
    if corners != None and ids != None:
      corners_list[0] = corners
      ids_list[0] = ids
  else:
    (corners2, ids2, rejected2) = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)
    if corners2 != None and ids2 != None:
      corners_list[1] = corners2
      ids_list[1] = ids2
  if ids_list[0] != None and ids_list[1] != None:
    zip1 = sorted(zip(ids_list[0], corners_list[0]))
    zip2 = sorted(zip(ids_list[1], corners_list[1]))
    for x in range(len(zip1)):
      if zip1[x][0] == zip2[x][0]:
        corners_1 = zip1[x][1][0][0]
        corners_2 = zip2[x][1][0][0]
        # print("top right corner 1 ", corners_1)
        # print("top right corner 2 ", corners_2)
        points3d = cv2.triangulatePoints(projMatr1 = proj1, projMatr2 = proj2, projPoints1 = corners_1, projPoints2 = corners_2)
        last_row = np.array([points3d[-1,:]]).T
        points3d = points3d/last_row
        print("triangulated points ", points3d)

        #center3d = centerPoint3d(corners_1, corners_2)
        # center3d = centerPoint3d(zip1[x][1], zip2[x][1])
        #print(np.array([center3d]).T)
  # rospy.sleep(1)

      
   

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('aruco_detector_py', anonymous=True)
  arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_7X7_50"])
  arucoParams = cv2.aruco.DetectorParameters_create()
  print("stuff")
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, callback, (0, arucoDict, arucoParams))

  rospy.Subscriber('/camera2/image_raw/compressed', CompressedImage, callback, (1, arucoDict, arucoParams))
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()
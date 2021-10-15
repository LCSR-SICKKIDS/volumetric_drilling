#!/usr/bin/python3

import h5py
import numpy as np
import rospy
import ros_numpy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from ambf_client import Client
import time
import math
import os
import re


bridge = CvBridge()

# Generate all objects in scene, even ones that may not be needed
_client = Client()
_client.connect() 
objects = {}

# Create python instances of AMBF objects
obj_names = _client.get_obj_names()
for obj_name in obj_names:
	obj_name = os.path.basename(os.path.normpath(obj_name)) # Get the last part of file path
	objects[obj_name]=_client.get_obj_handle(obj_name)
del objects['World']

locals().update(objects) # Add objects to global variables



# Create hdf5 file with date
if not os.path.exists('data'): os.makedirs('data')
timestr = time.strftime("%Y%m%d_%H%M%S")
f = h5py.File("./data/" + timestr + ".hdf5", "w")



# Get camera instrinsics from segmentation_camera ADF. Note that baseline is being read from
# camera_sync.py file, so if you're changing the baseline make sure to do so in camera_sync.py file and run
# the file to update the stereo ADF

f1 = open("../ADF/segmentation_camera.yaml", "r")
param = dict.fromkeys(["field view angle:", "publish image resolution:"])

for ln in f1:
	for key in param:
		if ln.lstrip().startswith(key): param[key] = ln[(ln.find(':')+1):].strip()
f1.close()

f2 = open("camera_sync.py")
for ln in f2: 
	if ln.lstrip().startswith("baseline"): 
		param["baseline"] = re.search(r'\d.+$', ln).group(0)
f2.close()

fva = float(param["field view angle:"])
img_height = float(re.search(r"height:\s*(\d*)\s*}", param["publish image resolution:"]).group(1))
img_width = float(re.search(r"width:\s*(\d+)\s*,", param["publish image resolution:"]).group(1))

focal = img_height/(2*math.tan(fva/2))
c_x = img_width/2
c_y = img_height/2

intrinsic = [[focal,0,c_x],[0,focal,c_y],[0,0,1]]

metadata = f.create_group("metadata")
metadata.create_dataset("camera_intrinsics",data = intrinsic)
metadata.create_dataset("baseline", data= param["baseline"])
metadata.create_dataset("README", data =  "All position information is in meters unless specified otherwise \n"
								  "Quaternion is a list in the order of [qx, qy, qz, qw] ")


# Store seq no initialized for each message, because in some cases seq no initialize differently
stereoL_no = None
stereoR_no = None
depth_no = None
segm_no = None


# The following are four callbacks for each of four subscribers created. As previously mentioned, sequence numbers
# sometimes dont line up so the first half of the callback is ensuring that each message is stored properly.
def stereoL_callback(image_msg):
	# This function is pulling stereoL image as well as object info for the scene
	global stereoL_no
	if stereoL_no is None: stereoL_no = image_msg.header.seq

	# Creates group for specific sequence no
	seq_no = image_msg.header.seq - stereoL_no
	if ("/" + str(seq_no)) not in f:
		curr_grp = f.create_group(str(seq_no))

	print("image:" +str(seq_no))

	curr_grp = f["/" + str(seq_no)]
	image_gen(image_msg,curr_grp,"stereoL")
	object_info(curr_grp)

def stereoR_callback(image_msg):
	global stereoR_no
	if stereoR_no is None: stereoR_no = image_msg.header.seq

	seq_no = image_msg.header.seq - stereoR_no
	if ("/" + str(seq_no)) not in f:
		curr_grp = f.create_group(str(seq_no))

	curr_grp = f["/" + str(seq_no)]
	image_gen(image_msg, curr_grp,"stereoR")

def depth_callback(depth_msg):
	global depth_no
	if depth_no is None: depth_no = depth_msg.header.seq

	seq_no = depth_msg.header.seq - depth_no
	if ("/" + str(seq_no)) not in f:
		curr_grp = f.create_group(str(seq_no))

	curr_grp = f["/" + str(seq_no)]	
	depth_gen(depth_msg,curr_grp)

def segm_callback(segm_msg):
	global segm_no
	if segm_no == None: segm_no = segm_msg.header.seq

	seq_no = segm_msg.header.seq - segm_no
	if ("/" + str(seq_no)) not in f:
		curr_grp = f.create_group(str(seq_no))

	curr_grp = f["/" + str(seq_no)]
	segm_gen(segm_msg, curr_grp)

def image_gen(image_msg,curr_grp,image_name):
	try:
		cv2_img = bridge.imgmsg_to_cv2(image_msg,"bgr8")
	except CvBridgeError as e:
		print(e)
	else:
		image_array = curr_grp.create_dataset(image_name,data= np.asarray(cv2_img))
		
def object_info(curr_grp):
	all_obj_grp = curr_grp.create_group("objects")
	for name, obj in objects.items():
		object_grp = all_obj_grp.create_group(name)
		
		pos_temp = [0,0,0]
		pos_temp[0]= obj.get_pos().x
		pos_temp[1]= obj.get_pos().y
		pos_temp[2]= obj.get_pos().z
		pos_array = object_grp.create_dataset("position", data=pos_temp)

		rot_temp = [0,0,0,0]
		rot_temp[0]= obj.get_rot().x
		rot_temp[1]= obj.get_rot().y
		rot_temp[2]= obj.get_rot().z
		rot_temp[3]= obj.get_rot().w
		rot_array = object_grp.create_dataset("quaternion", data=rot_temp)

def depth_gen(depth_msg,curr_grp):
	xyzd_array = ros_numpy.point_cloud2.pointcloud2_to_array(depth_msg)
	names = list(xyzd_array.dtype.names)
	names.remove('rgb') #rgb is redundant as it's alreayd in the image
	xyz_array = xyzd_array[names]

	depth_array = curr_grp.create_dataset("depth",data= xyz_array)

def segm_gen(segm_msg, curr_grp):
	cv2_img = bridge.imgmsg_to_cv2(segm_msg,"bgr8")
	image_array = curr_grp.create_dataset("segm_image",data= cv2_img)

def main():
	print("started")
	stereoL_topic = "/ambf/env/cameras/stereoL/ImageData"
	stereoR_topic = "/ambf/env/cameras/stereoR/ImageData"
	depth_topic = "/ambf/env/cameras/segmentation_camera/DepthData"
	segm_topic = "/ambf/env/cameras/segmentation_camera/ImageData"
	
	rospy.Subscriber(stereoL_topic, Image, stereoL_callback)
	rospy.Subscriber(stereoR_topic, Image, stereoR_callback)
	rospy.Subscriber(depth_topic, PointCloud2, depth_callback)
	rospy.Subscriber(segm_topic, Image, segm_callback)

	# # Moves the camera in a straight line
	# path_x = np.linspace(0.0, 0.4, num=500, endpoint=True)
	# path_y = np.linspace(0.0, 0.4, num=500, endpoint=True)
	# path_z = np.linspace(0.0, 0.2, num=500, endpoint=True)
	#
	# for i in range(len(path_x)):
	# 	temp_pos = [path_x[i], path_y[i], path_z[i]]
	# 	print(temp_pos)
	# 	objects['main_camera'].set_pos(temp_pos[0], temp_pos[1], temp_pos[2])  # Set the force in the World frame
	# 	time.sleep(0.04)  # Sleep for a while to see the effect of the command before moving on

	# # Moves the camera in a helical pattern
	# t = np.linspace(0.0, 6.28, num=500, endpoint=True)
	# path_x = 0.2*np.cos(t)
	# path_y = 0.4*np.sin(t)
	# path_z = 0.05*t
	#
	# for i in range(len(path_x)):
	# 	temp_pos = [path_x[i], path_y[i], path_z[i]]
	# 	print(temp_pos)
	# 	objects['main_camera'].set_pos(temp_pos[0], temp_pos[1], temp_pos[2])  # Set the force in the World frame
	# 	time.sleep(0.40)  # Sleep for a while to see the effect of the command before moving on

	rospy.spin()

	_client.clean_up()

if __name__ == '__main__':
	main()	


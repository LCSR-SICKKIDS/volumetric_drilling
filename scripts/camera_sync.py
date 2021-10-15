# This file can be used to sync camera parameters for cameras being used. Desired camera parameters should be
# placed in the segmenation_camera.yaml ADF and the desired baseline should be edited in this camera_sync file on
# line 8. When the program is run, stereo_cameras.yaml and single_stereo_camera.yaml will be edited to reflect 
# the same parameters used in segmentation_camera.yaml

import re

fi = open("../ADF/segmentation_camera.yaml", "r")
param = dict.fromkeys(["location:", "look at:", "clipping plane:", "field view angle:", "publish image resolution:", "publish image interval:"])
baseline = 0.065

for ln in fi:
	for key in param:
		if ln.lstrip().startswith(key): param[key] = ln[(ln.find(':')+1):].strip()
fi.close()


def sync(file):
	fo_t = open(file,"r")
	data = fo_t.readlines()
	fo_t.close()

	for count,ln in enumerate(data):
		for key in param:
			if ln.lstrip().startswith(key):
				data[count] = "  " + key + " " + param[key] + "\n"

	for count,ln in enumerate(data):
		if ln.lstrip().startswith("stereoR:"):
			iter = 0
			for index, ln2 in enumerate(data[count:]):
				if ln2.strip().startswith("location:"):
					rstereo_loc = re.search(r"y:(.*?),", ln2).group(1)
					rstereo_loc = float(rstereo_loc)
					adj_rstereo_loc = rstereo_loc + baseline
					# print(data[count+index])
					data[count+index] = re.search(r"^.*?y: ",ln2).group(0)+ str(adj_rstereo_loc) +\
						", " + re.search(r"z.*$",ln2).group(0) +"\n"
					# print(data[count+index])
					iter += 1
				if ln2.strip().startswith("look at:"):
					rstereo_la = re.search(r"y:(.*?),", ln2).group(1)
					rstereo_la = float(rstereo_la)
					adj_rstereo_la = rstereo_la + baseline
					# print(data[count+index])
					data[count+index] = re.search(r"^.*?y: ",ln2).group(0)+ str(adj_rstereo_la) +\
						", " + re.search(r"z.*$",ln2).group(0) +"\n"
					# print(data[count+index])
					iter += 1
				if iter == 2: break

					
					
	fo = open(file,"w")
	fo.writelines(data)
	fo.close()


sync("../ADF/stereo_cameras.yaml")
sync("../ADF/single_stereo_camera.yaml")
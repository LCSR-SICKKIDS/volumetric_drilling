import re

fi = open("segmentation_camera.yaml", "r")
param = dict.fromkeys(["location:", "look at:", "clipping plane:", "field view angle:", "publish image resolution:"])
baseline = 0.065

for ln in fi:
	for key in param:
		if ln.lstrip().startswith(key): param[key] = ln[(ln.find(':')+1):].strip()



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


sync("stereo_cameras.yaml")
sync("single_stereo_camera.yaml")
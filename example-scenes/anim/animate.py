def main():
	template = open("template").read()
	mtl_template = open("models/mtl.template").read()
	obj_template = open("models/obj.template").read()

	# keyframes = [
	# 	((0, 6.5, 20, 0, 0, 0, 1.58, -1), 0),
	# 	((0, 6.5, 20, 0, 13, 0, 1.58, -1), 40)
	# 	#((-1, 1, 1, .5, .5, .5, 1.58, -1), 10),
	# ]
	keyframes = [
		((0, 1, 4, 0, 1, 0, 1.58, -1), 0),
		((0, 1, 2, 0, 1, 0, 1.58, -1), 10),
		((0, 1, 2, 0, 1, 0, 1, -1), 10),
		((0, 1, 2, 0, 1, 0, 1, 1), 1),
		((0, 1, 2, 0, 1, 0, 1.5, 1), 10),
		((0, 1, 2, 0, 1, 0, 1.5, 1), 10),
		((-1, 1, 1, .5, .5, .5, 1.58, -1), 10)
	]


	frames = [keyframes[0][0]]
	for i in range(1, len(keyframes)):
		target = keyframes[i][0]
		prev = keyframes[i - 1][0]
		num_frames = keyframes[i][1]

		cur_frames = [make_cur_frame(i,target,prev,num_frames) for i in range(num_frames)]

		for f in cur_frames:
			frames.append(f)

	framedata = open("frames.out", "w")
	for frame in frames:
		framedata.write(str(frame) + "\n")
	for i, state in enumerate(frames):
		frame = str(template)
		obj = str(obj_template)
		mtl = str(mtl_template)
		x = state[0]
		y = state[1]
		z = state[2]
		look_x = state[3]
		look_y = state[4]
		look_z = state[5]
		light_y = state[6]
		light_y_norm = state[7]
		frame = frame.replace("{{OBJ_FILEPATH}}", "models/{}.obj".format(i))
		frame = frame.replace("{{X}}", "{:.4f}".format(x))
		frame = frame.replace("{{Y}}", "{:.4f}".format(y))
		frame = frame.replace("{{Z}}", "{:.4f}".format(z))
		frame = frame.replace("{{LOOK_X}}", "{:.4f}".format(look_x))
		frame = frame.replace("{{LOOK_Y}}", "{:.4f}".format(look_y))
		frame = frame.replace("{{LOOK_Z}}", "{:.4f}".format(look_z))
		frame = frame.replace("{{LOOK_Z}}", "{:.4f}".format(look_z))

		obj = obj.replace("{{MTL_FILEPATH}}", "{}".format(i))
		obj = obj.replace("{{Y}}", "{:.4f}".format(light_y))
		obj = obj.replace("{{Y_NORMAL}}", "{:.4f}".format(light_y_norm))

		outxml = open("{}.xml".format(i), "w+")
		outxml.write(frame)

		outobj = open("models/{}.obj".format(i), "w+")
		outobj.write(obj)

		outmtl = open("models/{}.mtl".format(i), "w+")
		outmtl.write(mtl)

def make_cur_frame(i, target, prev, num_frames):
	f_num_frames = float(num_frames)
	output = []
	for j in range(len(target)):
		output.append(prev[j] + (i/f_num_frames)*(target[j] - prev[j]))
	return tuple(output)

main()
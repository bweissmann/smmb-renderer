template = open("template").read()

# states = [(1,2), (1.1, 1.9), (1.2, 1.8), (1.2, 1.7), (1.3, 1.6), (1.3, 1.5), (1.3, 1.4), (1.3, 1.3)]

num_frames = 20
f_num_frames = float(num_frames - 1)
frames = [(1.0 + (i*0.3/f_num_frames), 2.0 - (i*0.7/f_num_frames)) for i in range(num_frames)]
framedata = open("frames.out", "w")
for frame in frames:
	framedata.write(str(frame) + "\n")
for i, yz in enumerate(frames):
	y = yz[0]
	z = yz[1]
	frame = template.replace("{{Y}}", "{:.4f}".format(y))
	frame = frame.replace("{{Z}}", "{:.4f}".format(z))

	outfile = open("{}.xml".format(i), "w+")
	outfile.write(frame)
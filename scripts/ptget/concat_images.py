import numpy as np
from PIL import Image
import sys

if len(sys.argv) != 3:
	print("Usage: " + sys.argv[0] + " [filename] [numfiles]")
	exit()

filename = sys.argv[1]
numfiles = int(sys.argv[2])
list_im = [filename + '_' + str(i) + '.png' for i in range(numfiles)]

imgs    = [ Image.open(i) for i in list_im ]
# pick the image which is the smallest, and resize the others to match it (can be arbitrary image shape here)
min_shape = sorted( [(np.sum(i.size), i.size ) for i in imgs])[0][1]  

# for a vertical stacking it is simple: use vstack
imgs_comb = np.vstack( (np.asarray( i.resize(min_shape) ) for i in imgs ) )
imgs_comb = Image.fromarray( imgs_comb)
imgs_comb.save( filename + '_full.png' )
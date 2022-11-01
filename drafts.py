import numpy as np
import re
with open('maps/brc202d.map') as f:
    lines = f.readlines()
    height = int(re.search(r'\d+', lines[1]).group())
    width = int(re.search(r'\d+', lines[2]).group())
    img_np = np.zeros((height, width))
    for height_index, line in enumerate(lines[4:]):
        for width_index, curr_str in enumerate(line):
            if curr_str == '.':
                img_np[height_index, width_index] = 1

    print(img_np)







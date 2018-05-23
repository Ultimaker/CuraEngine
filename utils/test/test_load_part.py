from helper.data_file_helper import DataFileHelper
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

storage, msg = DataFileHelper.load_part_file('/tmp/parts_by_layers.txt')
print(msg)

fig = plt.figure()
ax = fig.add_subplot(111)
parts = storage.get_volume(0).get_layer(3)

all_x = list()
all_y = list()
lines = list()
for part_nr in range(parts.get_parts_size()):
    for outline_nr in range(parts.get_part(part_nr).get_outlines_size()):
        x = parts.get_part(part_nr).get_outline(outline_nr).get_all_points_at_x()
        y = parts.get_part(part_nr).get_outline(outline_nr).get_all_points_at_y()
        x = [e/1000 for e in x]
        y = [e/1000 for e in y]
        all_x = all_x + x
        all_y = all_y + y
        # end point connects to begin point
        x.append(x[0])
        y.append(y[0])
        lines.append(Line2D(x, y))


for line in lines:
    ax.add_line(line)

ax.set_xlim(0, max(all_x))
ax.set_ylim(0, max(all_y))
plt.gca().set_aspect('equal', adjustable='box')
print("max x %d, max y %d" % (max(all_x), max(all_y)))

plt.show()
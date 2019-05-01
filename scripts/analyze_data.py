import os.path as osp

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

pkg_dir = '/Users/ebronstein/Documents/spring2019/ee106b/labs/lab4/lab4_pkg/'
cmd = 150.0
filename = '{0}.csv'.format(cmd)
filepath = osp.join(pkg_dir, 'data/bend_calibration/', filename)
df = pd.read_csv(filepath)

plt.scatter(df['tip_pos_x'], df['tip_pos_y'])
# plt.xlim((min(df['tip_pos_x']), max(df['tip_pos_x'])))
# plt.ylim((min(df['tip_pos_y']), max(df['tip_pos_y'])))
plt.show()
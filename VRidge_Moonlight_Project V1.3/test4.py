import time
import _thread
import matplotlib.pyplot as plt
import numpy as np

def plotting_thread(fig, axe):
    while (True):
        mat = np.random.randn(256, 256)
        time.sleep(0.01)  # ... or some busy computing
        axe.clear()
        axe.imshow(mat)
        fig.canvas.draw_idle()  # use draw_idle instead of draw

fig = plt.figure()  # the figure will be reused later
axe = fig.add_subplot(111)

_thread.start_new_thread(plotting_thread, (fig, axe))

plt.show()
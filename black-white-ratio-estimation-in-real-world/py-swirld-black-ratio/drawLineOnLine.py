import time
from matplotlib import pyplot as plt
import numpy as np

def live_update_demo(blit = False):
    x = np.linspace(0,50., num=100)
    X,Y = np.meshgrid(x,x)
    fig = plt.figure()
    
    ax1 = fig.add_subplot(2, 1, 1) # subplot 1 
    ax2 = fig.add_subplot(2, 1, 2) # subplot 2

    # #--- if you want to draw all lines to e.g., ax2, use the following, and delete ax1 
    # line1, = ax2.plot([], lw=3) 
    # line2, = ax2.plot([], lw=3)
    
    #--- if you want to draw lines on different subplots, use the followings.
    line1, = ax1.plot([], lw=3) 
    line2, = ax2.plot([], lw=3)
         
    ax1.set_xlim(x.min(), x.max())
    ax1.set_ylim([-1.1, 1.1])

    ax2.set_xlim(x.min(), x.max())
    ax2.set_ylim([-1.1, 1.1])

    fig.canvas.draw()   # note that the first draw comes before setting data 

    if blit:
        # cache the background
        axbackground = fig.canvas.copy_from_bbox(ax1.bbox)
        ax2background = fig.canvas.copy_from_bbox(ax2.bbox)

    plt.show(block=False)


    t_start = time.time()
    k=0.
    
    #-------------------------------------------------------------------------
    #----- plot the data in the for loop, you may change it to while loop
    #-------------------------------------------------------------------------

    for i in np.arange(1000):
        #----------------------------------------------------------------------
        line1.set_data(x, 2*x) # pass you data to line1 
        line2.set_data(x, np.cos(x/3.+k)) # pass you data to line2 
        #----------------------------------------------------------------------
        
        #print tx
        k+=0.11
        
        if blit:
            # restore background
            fig.canvas.restore_region(axbackground)
            fig.canvas.restore_region(ax2background)

            # redraw just the points
            ax1.draw_artist(line1) 
            ax2.draw_artist(line2)


            # fill in the axes rectangle
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)


        else:
            # redraw everything
            fig.canvas.draw()

        fig.canvas.flush_events()


live_update_demo(True)   # 175 fps
#live_update_demo(False) # 28 fps
import numpy as np
import matplotlib.pyplot as plt

def showplot(image_height,image_width,coords):
    plt.figure(figsize=(image_width / 100, image_height / 100), dpi=100)

    plt.scatter(coords[:,0],coords[:,1])
    plt.title("Segmented Boundary Outline")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    # print('image width:',image_width,'image height:',image_height)
    plt.xlim(0, image_width)
    plt.ylim(0, image_height)

    plt.gca().invert_yaxis()

    plt.show()

def matplot(mask_object,image_height,image_width):
    mask = mask_object.cpu().data[0].numpy()

    c=mask_object.xyn
    d=np.array(c,dtype=np.float32)
    co=np.vstack(d)
    coo = np.column_stack((co[:,0]*image_width,co[:,1]*image_height))
    coords=np.array(coo,dtype=np.uint32)

    showplot(image_height,image_width,coords)

    return coords

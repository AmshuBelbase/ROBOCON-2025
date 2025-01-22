import cv2
import numpy as np

def masking(result):
    mask_object = result[0].masks[result[0].boxes.cls==0]
    mask = mask_object.cpu().data[0].numpy()
    return mask,mask_object
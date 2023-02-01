from pyrsistent import b
import torch
from bbox import Bbox
from yolov5.detect import run as model



class Segmentation():

    def __init__(self) -> None:
        pass
    
    # def infer(self,img, threshold):
    def infer(self,img, threshold):
        # Model
        
        # Online model 
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
        results = model(img)
        
        # Retrain model
        # result = model(img = img, imgsz=(img.shape[0],img.shape[1])) # (height, width)
        # result = model(img = img) # (height, width)
        # return result
        infer_results = []
        for index, bbox_info in enumerate(results.xyxy):
            for bi in bbox_info:
                #print("bbox info",bi)
                # debug
                
                # similar to [1.31819e+03, 3.10094e+02, 1.40895e+03, 8.13340e+02, 6.58079e-01, 7.30000e+01]

                if bi[5] == 73 and bi[4] >= threshold: # in book categories
                    infer_results.append(self.yolo_scale2_pixel(bi)) 

        return infer_results                

    @staticmethod
    def yolo_scale2_pixel(bbox_info):
        
        x_min = bbox_info[0].cpu().numpy().tolist()
        y_min = bbox_info[1].cpu().numpy().tolist()
        x_max = bbox_info[2].cpu().numpy().tolist()
        y_max = bbox_info[3].cpu().numpy().tolist()
        return [(x_min,y_min),(x_min,y_max),(x_max,y_max),(x_max,y_min)]
        #return  Bbox(x_min,y_min,x_max,y_max)

        
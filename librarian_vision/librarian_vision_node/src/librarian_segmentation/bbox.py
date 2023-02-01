from tkinter.messagebox import NO


class Bbox():

    def __init__(self,x_min,y_min,x_max,y_max) -> None:
        self.left_top_x = x_min 
        self.left_top_y = y_min
        self.right_top_x = x_max 
        self.right_top_y = y_min 
        self.left_bottom_x = x_min 
        self.left_bottom_y = y_max 
        self.right_bottom_x = x_max 
        self.right_bottom_y = y_max 
    
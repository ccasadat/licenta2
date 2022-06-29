
import board
import pwmio
from adafruit_motor import servo

import os
import argparse
import cv2
import os
import numpy as np
import tkinter as tk
from tkinter import *
from PIL import Image, ImageTk
from sort import *
global mot_tracker
mot_tracker = Sort()

from pycoral.adapters.common import input_size
from pycoral.adapters.detect import get_objects
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from time import perf_counter_ns

class tinta():
    def __init__(self,id,tip,coords):
        self.id=id
        self.tip=tip
        self.coords=coords
global lista
lista=[]



def main():

    t1_stop=perf_counter_ns()

    root=Tk()
    latime=root.winfo_screenwidth()
    inaltime=root.winfo_screenheight()
    #latime=latime-256
    #inaltime=inaltime-120
    root.geometry("%dx%d" % (latime,inaltime))
    root.configure(bg="#2d2d2c")
    root.attributes('-zoomed',True)
    root.overrideredirect(True)
    root.attributes('-fullscreen',True)
    #servo
    #creare PWMOut object on pin PWM3
    pwm1 = pwmio.PWMOut(board.PWM2, duty_cycle=2 ** 15, frequency=50)
    pwm2 = pwmio.PWMOut(board.PWM1, duty_cycle=2 ** 15, frequency=50)
    #creare obiect 1
    servo1=servo.Servo(pwm1)
    servo2=servo.Servo(pwm2)


    #tkinter + servos commands
    global viteza1
    global viteza
    viteza=IntVar()
    global vertical_angle
    global horizontal_angle
    global x_click
    global y_click
    global flag_mod
    flag_mod=0
    global flag_click
    flag_click=0
    global flag_tinta
    flag_tinta=0
    vertical_angle=90
    horizontal_angle=90
    servo1.angle=vertical_angle
    servo2.angle=horizontal_angle

    viteza.set("3")

    def speed():
        global viteza1
        viteza1=viteza.get()
    speed()

    def cmd_dreapta():
        print("dreapta")
        global viteza1
        global horizontal_angle
        horizontal_angle=horizontal_angle-viteza1
        if horizontal_angle > 180:
            horizontal_angle=180
        servo2.angle=horizontal_angle


    def cmd_stanga():
        print("stanga")
        global viteza1
        global horizontal_angle
        horizontal_angle=horizontal_angle+viteza1
        if horizontal_angle < 0:
            horizontal_angle=0
        servo2.angle=horizontal_angle


    def cmd_sus():
        print("sus")
        global viteza1
        global vertical_angle
        vertical_angle=vertical_angle-viteza1
        if vertical_angle > 180:
            vertical_angle=180
        servo1.angle=vertical_angle


    def cmd_jos():
        print("jos")
        global viteza1
        global vertical_angle
        vertical_angle=vertical_angle+viteza1
        if vertical_angle < 0:
            vertical_angle=0
        servo1.angle=vertical_angle


    def cmd_center():
        global vertical_angle
        global horizontal_angle
        vertical_angle=90
        horizontal_angle=90
        servo1.angle=vertical_angle
        servo2.angle=vertical_angle

    def shutdown():
        top1= Toplevel()
        top1.geometry("490x120+30+40")
        top1.title("Inchidere")
        text=Label(top1, text= " \n Sunteti sigur ca doriti sa inchideti? ", font=('DejavuSans',40))
        text.grid(row=3,column=1,rowspan=1,columnspan=4)
        #top1.configure(bg="#2d2d2c")
        #top.overrideredirect(True)
        #top1.attributes('-zoomed',True)
        top1.attributes("-topmost", True)

        def close_prompt():
            top1.destroy()

        def shutdown():
            return os.system("shutdown now")

        button_shutdown_now=Button(top1, bg="red", text="Inchide", font=('DejavuSans',20),command=shutdown)
        button_shutdown_now.grid(row=4,column=1,sticky='ewns')
        button_close_prompt=Button(top1, bg="blue", text="Anuleaza", font=('DejavuSans',20),command=close_prompt)
        button_close_prompt.grid(row=4,column=4,sticky='ewns')



    def open_help():
        top= Toplevel()
        #root.geometry("%dx%d" % (latime,inaltime))
        top.title("Instructiuni")
        text=Label(top, text= "Instructiunile se scriu aici'\n' wpw ", font=('DejavuSans',40))
        text.grid(row=1,column=2,rowspan=2)
        top.configure(bg="#2d2d2c")
        top.attributes('-zoomed',True)
        #top.overrideredirect(True)
        #top.attributes('-fullscreen',True)
        top.attributes("-topmost", True)

        def close_help():
            top.destroy()



        button_close_help=Button(top, bg="blue", text="inchide", font=('DejavuSans',20),command=close_help)
        button_close_help.grid(row=3,column=1)





    def mod_interior():
        print("interior")
        global flag_mod
        flag_mod=1


    def mod_exterior():
        print("exterior")
        global flag_mod
        flag_mod=0

    def click(e):
        global x_click
        global y_click
        global flag_click
        flag_click=1
        x_click=e.x
        y_click=e.y
        print("mouse at %d and %d" %(x_click,y_click))

    def deselectare():
        global flag_tinta
        flag_tinta=0

    def exit_program():
        root.destroy()





    button_right=Button(root,bg="#0e8ad6", text="►", font=('DejavuSans',25),command=cmd_dreapta,repeatinterval=5,repeatdelay=1, relief="raised")
    button_right.grid(row=2, column=13, sticky='ewns', ipadx=8)

    button_left=Button(root,bg="#0e8ad6", text="◄", font=('DejavuSans',25),command=cmd_stanga,repeatinterval=5,repeatdelay=1, relief="raised")
    button_left.grid(row=2, column=11, sticky='ewns', ipadx=8)

    button_up=Button(root,bg="#0e8ad6", text="▲", font=('DejavuSans',25),command=cmd_sus,repeatinterval=5,repeatdelay=1, relief="raised")
    button_up.grid(row=1, column=12, sticky='ewns', ipadx=11)

    button_down=Button(root,bg="#0e8ad6", text="▼", font=('DejavuSans',25),command=cmd_jos,repeatinterval=5,repeatdelay=1, relief="raised")
    button_down.grid(row=3, column=12, sticky='ewns', ipadx=11)

    button_quit=Button(root,bg="red", text="Exit", font=('DejavuSans',25),command=exit_program, relief="raised")
    button_quit.grid(row=11,column=12,padx=1,columnspan=1)

    button_center=Button(root,bg="#0e8ad6", text="◎", font=('DejavuSans',35),command=cmd_center, relief="raised")
    button_center.grid(row=2, column=12,sticky='ewns')

    button_help=Button(root,bg="#0e8ad6", text="Instructiuni", font=('DejavuSans',20),command=open_help, relief="raised")
    button_help.grid(row=11, column=1)

    button_shutdown=Button(root,bg="red",text="Inchidere",font=('DejavuSans',20),command=shutdown, relief='raised')
    button_shutdown.grid(row=11, column=2, sticky='ewns')

    button_interior=Button(root,bg="#0e8ad6", text="Interior",font=('DejavuSans',20),command=mod_interior, relief='raised')
    button_interior.grid(row=11, column=3, sticky='ewns')

    button_exterior=Button(root,bg="#0e8ad6", text="Exterior",font=('DejavuSans',20),command=mod_exterior, relief='raised')
    button_exterior.grid(row=11, column=4,sticky='ewns')

    button_deselectare=Button(root,bg="#0e8ad6", text="Deselectare",font=('DejavuSans',20),command=deselectare, relief='raised')
    button_deselectare.grid(row=11, column=5,sticky='ewns')


    L2=Label(root,text="Viteza camera:",font=('DejavuSans',20))
    L2.grid(row=4,column=11,columnspan=3, sticky='ewns')



    button_speed_1=Radiobutton(root,bg="blue", text="1", font=('DejavuSans',20),indicator=0,variable=viteza,value=1,command=speed)
    button_speed_1.grid(row=5, column=11,sticky='ewns')

    button_speed_2=Radiobutton(root,bg="blue", text="2", font=('DejavuSans',20),indicator=0,variable=viteza,value=3,command=speed)
    button_speed_2.grid(row=5, column=12,sticky='ewns')

    button_speed_3=Radiobutton(root,bg="blue", text="3", font=('DejavuSans',20),indicator=0,variable=viteza,value=10,command=speed)
    button_speed_3.grid(row=5, column=13,sticky='ewns')


    L1=Label(root,bg="red")
    L1.grid(row=1, column=1,rowspan=10,columnspan=10)
    L1.bind("<Button-1>",click)


    default_model_dir = '../all_models'
    default_model = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
    default_labels = 'coco_labels.txt'
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', help='.tflite model path',
                        default=os.path.join(default_model_dir,default_model))
    parser.add_argument('--labels', help='label file path',
                        default=os.path.join(default_model_dir, default_labels))
    parser.add_argument('--top_k', type=int, default=3,
                        help='number of categories with highest score to display')
    parser.add_argument('--camera_idx', type=int, help='Index of which video source to use. ', default = 0)
    parser.add_argument('--threshold', type=float, default=0.1,
                        help='classifier score threshold')
    args = parser.parse_args()

    print('Loading {} with {} labels.'.format(args.model, args.labels))
    interpreter = make_interpreter(args.model)
    interpreter.allocate_tensors()
    labels = read_label_file(args.labels)
    inference_size = input_size(interpreter)


    t1_start=perf_counter_ns()

    cap = cv2.VideoCapture(0)

    print("??????????????????????????????????????????????????????????????")

    while True:

        t1_while=perf_counter_ns()

        ret, frame = cap.read()
        if not ret:
            break
        cv2_im = frame

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        cv2_im_rgb = cv2.resize(cv2_im_rgb,inference_size)
        run_inference(interpreter, cv2_im_rgb.tobytes())
        objs = get_objects(interpreter, args.threshold)[:args.top_k]


        boxes=[]
        global track_bbs_ids
        track_bbs_ids=[]
        for obj in objs:
            boxes.append(np.array([obj.bbox.xmin, obj.bbox.ymin, obj.bbox.xmax, obj.bbox.ymax ,obj.score, obj.id]))
        if objs:
            track_bbs_ids = mot_tracker.update(np.array(boxes))

        cv2_im = append_objs_to_img(cv2_im, inference_size, objs, labels, track_bbs_ids)
        img=cv2.cvtColor(cv2_im,cv2.COLOR_BGR2RGB)


        #print(t1_start-t1_stop)
        global lista

        if flag_click:
            global x_click
            global y_click
            distance_final=99999
            for i in range(len(lista)):
                x,y=lista[i].coords
                distance=np.sqrt((x_click-x)**2+(y_click-y)**2)
                if distance < distance_final:
                    distance_final=distance
                    global target
                    target=lista[i].id
            flag_tinta=1
            flag_click=0
        if flag_tinta:
            for i in range(len(lista)):
                if lista[i-1].id ==target:
                    a,b=lista[i-1].coords
                    print("am tinta")
                    if a <= 192:
                        horizontal_angle=horizontal_angle+5
                        if horizontal_angle>180:
                            horizontal_angle=180
                        servo2.angle=horizontal_angle

                    if a >384:
                        horizontal_angle=horizontal_angle-5
                        if horizontal_angle<0:
                            horizontal_angle=0
                        servo2.angle=horizontal_angle

                    if b <=400:
                        vertical_angle=vertical_angle-3
                        if vertical_angle<0:
                            vertical_angle=0
                        servo1.angle=vertical_angle

                    if b > 220:
                        vertical_angle=vertical_angle+3
                        if vertical_angle>180:
                            vertical_angle=180
                        servo1.angle=vertical_angle
                else:
                    flag_tinta=0
                    print("nu mai am tinta")

        lista=[]

        t1=perf_counter_ns()

        scale_percent = 90 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        print(dim)
        img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

        img=ImageTk.PhotoImage(Image.fromarray(img))
        L1['image']=img
        root.update()

        t2=perf_counter_ns()
        #print(t2-t1)

        t2_while=perf_counter_ns()
        print(t2_while-t1_while)

    cap.release()
    cv2.destroyAllWindows()



def append_objs_to_img(cv2_im, inference_size, objs,labels, track_bbs_ids):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[0], height / inference_size[1]
    for lines in range(len(track_bbs_ids)):
        x0, y0 = int(scale_x*track_bbs_ids[lines-1][0]), int(scale_y*track_bbs_ids[lines-1][1])
        x1, y1 = int(scale_x*track_bbs_ids[lines-1][2]), int(scale_y*track_bbs_ids[lines-1][3])

        #print(track_bbs_ids[lines-1][5])
        label = '{}% {}'.format( labels.get(track_bbs_ids[lines-1][5]),track_bbs_ids[lines-1][4])
        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

        centroid=[(x0+x1)/2,(y0+y1)/2]
        buffer=tinta(track_bbs_ids[lines-1][4],labels.get(track_bbs_ids[lines][5]),centroid)
        global lista
        lista.append(buffer)

        if flag_mod==1:
            global flag_tinta
            if flag_tinta==1:
                print("inca am tinta")
                break
            else:
                for i in range(len(lista)):
                    if lista[i-1].tip=='persoana':
                        global target
                        target=lista[i].id
                        flag_tinta=1
                        print("am tinta persoana noua")



    return cv2_im

if __name__ == '__main__':
    main()



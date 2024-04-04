import cv2
import threading
import time
import argparse
from queue import LifoQueue,Queue
import logging
from typing import List
import os

class Sensor:
    def get(self):
        raise NotImplementedError("subclasses must implement method get()")



class SensorX(Sensor):
    """Sensor X"""
    def __init__(self, delay: float):
        self.delay = delay
        self.data = 0

    def get(self) -> int:
        time.sleep(self.delay)
        self.data += 1
       # print("и счётчик тоже чето делает")
        return self.data


class SensorCam(Sensor):
    def __init__(self,camera_index:int = 0 ,width:int = 640,height:int = 480):
        global bigerror
        try:
            self._cap = cv2.VideoCapture(camera_index)
            if not self._cap.isOpened():
                logging.error("we dont have a camera with this camera index")
                bigerror = True
                self.stop()
                
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
        except Exception as e:
            logging.error("We have an error : %s",str(e))
            bigerror = True
            self.stop()


    
    def get(self):
        ret,frame = self._cap.read()
        if not ret:
            logging.error("we have no img from cam")
            global bigerror
            bigerror = True
            self.stop()
        return frame

    def stop(self):
        self._cap.release()
        

    def __del__(self):
        self.stop()

def sensor_worker(sensor:SensorX,queue:LifoQueue):
    global flag
    lock = threading.Lock()
    
    while flag:
        a = sensor.get()
        
        if queue.full():
            queue.get()
            queue.put(a)
        else:
            queue.put(a)
        
    print(" я всё ")

    

def camera_worker(queue:Queue):
    global flag
    cam = SensorCam()
    while flag:
        a = cam.get()
        if queue.full():
            queue.get()
            queue.put(a)
        else:
            queue.put(a)
        
       # print(" i put frame")
    cam.stop

class ImageWindow():
    def __init__(self,fps:int = 15,height:int = 480):
        self._sensor_data = [0,0,0]
        self.frame = None
        self.fps = fps
        self._lock = threading.Lock()
        self._height = height
    def show(self,cam_queue:Queue,queues:List[Queue]):
        try:
            
            for i in range(3):
                if queues[i].full():
                    self._sensor_data[i] = queues[i].get()
                    print(self._sensor_data[i])
            if cam_queue.full():
                self.frame=cam_queue.get()
            
            cv2.putText(self.frame,f"Sensor1: {self._sensor_data[0]}  Sensor2: {self._sensor_data[1]}  Sensor3: {self._sensor_data[2]}", (10,self._height-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1)
            cv2.imshow('camera and data',self.frame)
        except Exception as e:
            logging.error("We have an error at show(): %s",str(e))

    def stop(self):
        cv2.destroyAllWindows()

    def __del__(self):
        self.stop()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ширина,высота,фпс")
    parser.add_argument("--camIndex",type=int,default=0)
    parser.add_argument("--height", type=int, default =480 )
    parser.add_argument("--width", type=int, default = 720)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()
    if not os.path.exists('log'):
        os.makedirs('log')
    
    log_file = os.path.join('log','error.log')
    logging.basicConfig(filename=log_file, level=logging.ERROR,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    flag = True
    bigerror = False
    # создаём сенсоры , камеру и окнонный обозреватель
    sensors = [SensorX(i) for i in [0.01,0.1,1]]
    sensor_queues = [LifoQueue(maxsize=1) for _ in range(3)]
    cam_queue = Queue(maxsize=1)
    sensor_workers = [threading.Thread(target=sensor_worker,args=(sensors[i],sensor_queues[i])) for i in range(3)]
    cam_worker = threading.Thread(target=camera_worker,args=(cam_queue,))
    time.sleep(1)
    window_imager = ImageWindow(fps = args.fps,height=args.height)
    for i in range(3):
        sensor_workers[i].start()
    cam_worker.start()
    while True:
        
        print("tryhard")
        window_imager.show(cam_queue,sensor_queues)
        if cv2.waitKey(1) & 0xFF == ord('q') or bigerror:
            window_imager.stop()
            
            flag=False
            cam_worker.join()
            for sensor_workerr in sensor_workers:
                sensor_workerr.join()
            break
       # print('nen')
        # сделаем задержку чтобы можно было показывать с задаваемым фпс
        
        time.sleep(1/window_imager.fps)
       # print('???')


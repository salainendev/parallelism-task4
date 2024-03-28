import cv2
import threading
import time
import argparse
from queue import Queue
import logging
from typing import List
import os

class Sensor:
    def get(self):
        raise NotImplementedError("subclasses must implement method get()")



class SensorX(Sensor):
    def __init__(self,delay:float):
        self._delay = delay
        self._data = 0
        self._running = True
        self.sensor_queue = Queue()
        self._lock = threading.Lock()
        self._sensor_thread = threading.Thread(target=self.__sensor_work)
        self._sensor_thread.start()
    
    def __sensor_work(self):
        while self._running:
            time.sleep(self._delay)
            self._data+=1
            with self._lock:
                self.sensor_queue.put(self._data)

    def get(self):
        with self._lock:
            res = self.sensor_queue.get()
        return res

    def stop(self):
        self._running=False
        self._sensor_thread.join()

    def __del__(self):
        self.stop()


class SensorCam(Sensor):
    def __init__(self,queue:Queue,camera_index:int = 0 ,width:int = 640,height:int = 480):
        try:
            self._cap = cv2.VideoCapture(camera_index)
            if not self._cap.isOpened():
                logging.error("we dont have a camera with this camera index")
                self.stop()
                
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
            self._lock = threading.Lock()
            self.output_queue = queue
            self._running = True
            self._camera_thread = threading.Thread(target=self.__camera_task)
            self._camera_thread.start()
        except Exception as e:
            self.stop()
            logging.error("We have an error : %s",str(e))


    def __camera_task(self):
        while self._running:
            ret,frame = self._cap.read()
            if not ret:
                logging.error("we have no img from webcam")
                self.stop()
                break
            with self._lock:
                self.output_queue.put(frame)
                

    

    def get(self):
        with self._lock:
            res = self.output_queue.get()
        return res

    def stop(self):
        self._running=False
        if hasattr(self, 'camera_thread'):
            self._camera_thread.join()
        self._cap.release()
        

    def __del__(self):
        self.stop()

class ImageWindow():
    def __init__(self,fps:int = 15):
        self._sensor_data1 = 0
        self._sensor_data2 = 0
        self._sensor_data3 = 0
        self.frame = None
        self.fps = fps
        
    def show(self,queue:Queue,sensor_cam:SensorCam,sensors: List[SensorX]):
        try:
            if not sensors[0].sensor_queue.empty():
                self.sensor_data1 = sensors[0].get()
            if not sensors[1].sensor_queue.empty():
                self.sensor_data2 = sensors[1].get()
            if not sensors[2].sensor_queue.empty():
                self.sensor_data3 = sensors[2].get()
            if not sensor_cam.output_queue.empty():
                self.frame = sensor_cam.get()
            
            cv2.putText(self.frame,f"Sensor1: {self.sensor_data1}  Sensor2: {self.sensor_data2}  Sensor3: {self.sensor_data3}", (10,self.frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1)
            cv2.imshow('camera and data',self.frame)
        except Exception as e:
            logging.error("We have an error at show(): %s",str(e))

    def stop(self):
        cv2.destroyAllWindows()

    def __del__(self):
        self.stop()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ширина,высота,фпс")
    parser.add_argument("--height", type=int, default =480 )
    parser.add_argument("--width", type=int, default = 720)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()
    if not os.path.exists('log'):
        os.makedirs('log')
    
    log_file = os.path.join('log','error.log')
    logging.basicConfig(filename=log_file, level=logging.ERROR,
                        format='%(asctime)s - %(levelname)s - %(message)s')

    # создаём сенсоры (потоки начинают работу при инициализации) , камеру и окнонный обозреватель
    sensors = [SensorX(i) for i in [0.01,0.1,1]]
    queue = Queue()
    sensor_cam = SensorCam(queue,width = args.width,height = args.height)
    time.sleep(1)
    window_imager = ImageWindow(fps = args.fps)
    while True:
        start_time = time.time()
        print("tryhard")
        window_imager.show(queue,sensor_cam,sensors)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            window_imager.stop()
            sensor_cam.stop()
            for sensor in sensors:
                sensor.stop()
            break
       # print('nen')
        # сделаем задержку чтобы можно было показывать с задаваемым фпс
        time_elapsed = time.time() - start_time
        time.sleep(1/window_imager.fps)
       # print('???')


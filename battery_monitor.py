import threading
import time

from adafruit_ina3221 import INA3221

MONITOR_INTERVAL_S = 5 # how often to check battery voltage
MIN_VOLTAGE = 10.6 # [V]

class BatteryMonitor:
    def __init__(
        self,
        ina: INA3221,
        ina_channel: int,
        battery_low_event: threading.Event,
        i2c_lock: threading.Lock,
    ):
        self.ina = ina
        self.ina_channel = ina_channel
        self.battery_low_event = battery_low_event
        self.i2c_lock = i2c_lock
        
        monitor_battery_thread = threading.Thread(target=self.monitor_battery)
        monitor_battery_thread.daemon = True
        monitor_battery_thread.start()
    
    def monitor_battery(self):
        while True:
            self.i2c_lock.acquire()
            self.ina.mode = 2
            time.sleep(0.02)

            try:
                if self.ina[self.ina_channel].bus_voltage < MIN_VOLTAGE:
                    self.battery_low_event.set()
                self.i2c_lock.release()
            except Exception as e:
                print(f"Exception during battery monitor thread: {e}")
                self.i2c_lock.release()
            else:
                time.sleep(MONITOR_INTERVAL_S)
            
            

            
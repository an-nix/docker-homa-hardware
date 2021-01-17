import smbus2
import bme280
import bme280.const as oversampling
import time
import json
import os
import logging


from threading import Thread
import paho.mqtt.client as mqtt

# Start measurement at 4lx resolution. Time typically 16ms.
CONTINUOUS_LOW_RES_MODE = 0x13
# Start measurement at 1lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_1 = 0x10
# Start measurement at 0.5lx resolution. Time typically 120ms
CONTINUOUS_HIGH_RES_MODE_2 = 0x11
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_1 = 0x20
# Start measurement at 0.5lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_HIGH_RES_MODE_2 = 0x21
# Start measurement at 1lx resolution. Time typically 120ms
# Device is automatically set to Power Down after measurement.
ONE_TIME_LOW_RES_MODE = 0x23

def getserial():
  # Extract serial from cpuinfo file
  cpuserial = "0000000000000000"
  try:
    f = open('/proc/cpuinfo','r')
    for line in f:
      if line[0:6]=='Serial':
        cpuserial = line[10:26]
    f.close()
  except:
    cpuserial = "ERROR000000000"

  return cpuserial

class MqttSensorBH1750:
    def __init__(self,address,illuminance_offset=0.0, round = 2):
        self.__bus = smbus2.SMBus(1)
        self.__address = address
        self.__illuminance_offset = illuminance_offset
        self.__round = round

    def convertToNumber(self,data):
        # Simple function to convert 2 bytes of data
        # into a decimal number. Optional parameter 'decimals'
        # will round to specified number of decimal places.
        result=(data[1] + (256 * data[0])) / 1.2
        return (result)

    def read(self):
        # Read data from I2C interface
        self.__data = self.__bus.read_i2c_block_data(self.__address,ONE_TIME_HIGH_RES_MODE_1,2)
        __dict = {"illuminance":round(self.convertToNumber(self.__data) + self.__illuminance_offset,self.__round)}
        #self.payload = json.dumps(__dict)
        self.payload = __dict
        

    def config_payload(self,name,unique_id,state_topic):
        ret = []
        ret.append({
                "device_class": "illuminance",
                "unique_id":"illu_{0}".format(unique_id),
                "name": "{0}.illuminance".format(name),
                "state_topic": state_topic,
                "unit_of_measurement": "lx",
                "value_template":"{{ value_json.illuminance}}",
                "device" : {
                    "identifiers":[unique_id],
                    "name":"Onboard Sensor",
                    "model":"HoMa",
                    "manufacturer":"meba.xyz"
                        }
                })
        return ret

class MqttSensorBME280:

    def __init__(self,address, temperature_offset=0.0, pressure_offset=0.0, humidity_offset=0.0, altitude=0.0, round = 2):
        self.__bus = smbus2.SMBus(1)
        self.__address = address
        self.__calibration_params = bme280.load_calibration_params(self.__bus, self.__address)
        self.__temperature_offset = temperature_offset
        self.__pressure_offset = pressure_offset
        self.__humidity_offset = humidity_offset
        self.__altitude = altitude
        self.__round = round
        
        pass

    def normalize_pressure(self, raw_pressure):
        
        np = raw_pressure*(1 - (0.0065*self.__altitude / 288.15))**-5.255
        return np

    def read(self):
        self.__data = bme280.sample(self.__bus, self.__address, self.__calibration_params,sampling=2)
        __dict = {
            "temperature": round(self.__data.temperature + self.__temperature_offset,self.__round),
            "humidity": round(self.__data.humidity + self.__humidity_offset,self.__round),
            "pressure": round(self.normalize_pressure(self.__data.pressure + self.__pressure_offset),self.__round) 
        }
        self.payload = __dict

    def config_payload(self,name,unique_id,state_topic):
        return [{
                "device_class": "temperature",
                "unique_id":"tem_{0}".format(unique_id),
                "name": "{0}.temperature".format(name),
                "state_topic": state_topic,
                "unit_of_measurement": "°C",
                "value_template":"{{ value_json.temperature}}",
                "device" : {
                    "identifiers":[unique_id],
                    "name":"Onboard Sensor",
                    "model":"HoMa",
                    "manufacturer":"meba.xyz"
                }
            },
            {
                "device_class": "humidity",
                "unique_id":"hum_{0}".format(unique_id),
                "name": "{0}.humidity".format(name),
                "state_topic": state_topic,
                "unit_of_measurement": "%",
                "value_template":"{{ value_json.humidity}}",
                "device" : {
                    "identifiers":[unique_id],
                    "name":"Onboard Sensor",
                    "model":"HoMa",
                    "manufacturer":"meba.xyz"
                }
            },   
            {
                "device_class": "pressure",
                "unique_id":"pres_{0}".format(unique_id),
                "name": "{0}.pressure".format(name),
                "state_topic": state_topic,
                "unit_of_measurement": "hPa",
                "value_template":"{{ value_json.pressure}}",
                "device" : {
                    "identifiers":[unique_id],
                    "name":"Onboard Sensor",
                    "model":"HoMa",
                    "manufacturer":"meba.xyz"
                }
            }]
 
class MqttSensors(Thread):

    def __init__(self, client):
        Thread.__init__(self)
        self.__mqtt_client = client
        self.__dicovery = False
        self.__unique_id = getserial()[8:]
        self.__temp = MqttSensorBME280(0x76)
        self.__sensor = []
    
    def register_sensor(self,sensor):
        self.__sensor.append(sensor)


    def release_autoconfig(self):
        if(self.__dicovery == True):
            self.discovery=False
            

    def auto_configuration(self):
        #<discovery_prefix>/<component>/[<node_id>/]<object_id>/config
        #Send discovery Topic on the first run
        if(self.__dicovery == False):
            #"homeassistant/sensor/onboard/state"
            ret = []
            
            for s in self.__sensor:
                ret.extend(s.config_payload("onboard",self.__unique_id,"homeassistant/sensor/onboard/state"))            

            for i in ret:
                logging.info(f"Disovery message: {json.dumps(i)}") 
                self.__mqtt_client.publish("homeassistant/sensor/onboard/{0}/config".format(i["device_class"]),json.dumps(i),retain=True)            

            self.__dicovery = True
            self.start()

    def run(self):
        while(self.__dicovery):
            #self.__temp.read()
            value = {}
            for s in self.__sensor:
                s.read()
                value.update(s.payload)
            self.__mqtt_client.publish("homeassistant/sensor/onboard/state",json.dumps(value))
            time.sleep(10)


def on_disconnect(client, userdata,rc=0):
    logging.info("DisConnected result code "+str(rc))
    mh.release_autoconfig()

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    logging.info("Connected with result code "+str(rc))
    mh.auto_configuration() 

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    logging.info(msg.topic+" "+str(msg.payload))

def main():
    
    logging.basicConfig(
    format='%(asctime)s;%(levelname)s;%(message)s',
    level=logging.INFO,
    datefmt='%Y-%m-%d %H:%M:%S')
    
    #Set Callback functions
    client.on_disconnect = on_disconnect
    client.on_connect = on_connect
    client.on_message = on_message


    #MQTT HOSTNAME
    if "MQTT_HOSTNAME" not in os.environ:
        mqtt_hostname = "localhost"
    else:
        mqtt_hostname = os.environ["MQTT_HOSTNAME"]

    #MQTT PORT
    if "MQTT_PORT" not in os.environ:
        mqtt_port = 1883
    else:
        mqtt_port = int(os.environ["MQTT_PORT"])

    if "MQTT_KEEPALIVE" not in os.environ:
        mqtt_keepalive = 60
    else:
        mqtt_keepalive = os.environ["MQTT_KEEPALIVE"]



    if "OFFSET_PRESSURE" not in os.environ:
        OFFSET_PRESSURE = 0
    else:
        OFFSET_PRESSURE = float(os.environ["OFFSET_PRESSURE"])

    if "OFFSET_HUMIDITY" not in os.environ:
        OFFSET_HUMIDITY = 0
    else:
        OFFSET_HUMIDITY = float(os.environ["OFFSET_HUMIDITY"])

    if "OFFSET_TEMPERATURE" not in os.environ:
        OFFSET_TEMPERATURE = 0
    else:
        OFFSET_TEMPERATURE = float(os.environ["OFFSET_TEMPERATURE"])

    if "OFFSET_ILLUMINANCE" not in os.environ:
        OFFSET_ILLUMINANCE = 0
    else:
        OFFSET_ILLUMINANCE = float(os.environ["OFFSET_ILLUMINANCE"])

    if "ALTITUDE" not in os.environ:
        ALTITUDE = 0
    else:
        ALTITUDE = float(os.environ["ALTITUDE"])
    
    if "ROUND" not in os.environ:
        ROUND = 2
    else:
        ROUND = int(os.environ["ROUND"])

    logging.info("Register BME280 Sensor")
    mh.register_sensor(MqttSensorBME280(0x76,temperature_offset=OFFSET_TEMPERATURE,
                                            pressure_offset=OFFSET_PRESSURE,
                                            humidity_offset=OFFSET_HUMIDITY,
                                            altitude=ALTITUDE,
                                            round=ROUND))

    logging.info("Register BH1750 Sensor")
    mh.register_sensor(MqttSensorBH1750(0x23,illuminance_offset=OFFSET_ILLUMINANCE,round=ROUND))


    logging.info(f"Connect to MQTT Broker {mqtt_hostname}:{mqtt_port}")

    client.connect_async(mqtt_hostname,mqtt_port,mqtt_keepalive)


    client.loop_start()
    
    while(1):
        time.sleep(10)


client = mqtt.Client()
mh = MqttSensors(client)

if __name__ == "__main__":
    # execute only if run as a script
    main()
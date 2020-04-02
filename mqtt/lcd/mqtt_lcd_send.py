#!/usr/bin/python
# send data to the LCD

import time
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
#import paho.mqtt as mqtt
import json
import socket
import fcntl
import struct
import psutil

MQTT_SERVER = "127.0.0.1"
MQTT_PATH = "LCD_channel"
SW_PATH = "SWITCH_in"
ADC_PATH = "adc_channel"
switch = {}
adc = {}
adc[0] ="xxx"
adc[1] ="xxx"
adc[2] ="xxx"
adc[3] ="xxx"
lcd_state = {}
lstat = 0;

def get_cpu_temp():
    foo = psutil.sensors_temperatures()
    for name,entries in foo.items():
        for entry in entries:
            os = (" %-20s %s DegC " %(entry.label or name, entry.current))
            return os

def get_cpu_time():
    return "Time: %s" % time.strftime("%H:%M:%S")

def get_cpu_date():
        return "Date: %s" %time.strftime("%m/%d/%Y")

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,
        struct.pack('256s', ifname[:15])
    )[20:24])


def lcd_send(path, line1, line2):
    msg = {}
    msg['key']= "lcd"
    msg['line1']= line1
    msg['line2']= line2
    jmsg = json.dumps(msg);
    publish.single(path, jmsg, hostname=MQTT_SERVER)


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe(MQTT_PATH1)
    #client.subscribe(MQTT_PATH2)
    client.subscribe(SW_PATH)
    client.subscribe(ADC_PATH)

 # The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global switch
    global adc
    global lcd_state
    global lstat
    jmes = json.loads(msg.payload)
    if msg.topic == SW_PATH:
        val  = jmes['value']
        ch  = jmes['chan']
        #lstat += 1
        print(msg.topic+" LCDxxs->"+str(msg.payload)+ "val:" + str(val))
        print(" key->"+jmes['key'])
        print(" chan->"+str(ch))
        print(" value->"+str(val))
        if ch == 17 and  val == 1:
            print ("inc lstat")
            lstat += 1
        if ch == 27 and val == 1:
            print ("dec lstat")
            lstat -= 1
        print(' Set new lstat 1 ' + str(lstat) )        
        if lstat < 0:
            lstat = 2
        if lstat > 2:
            lstat = 0
        print(' Set new lstat 2 ' + str(lstat) + ' lcd_state: ' + lcd_state[lstat])        
        #if msg.topic == SW_PATH: 
        #if jmes['key'] == 'switch':
        
        switch[jmes['chan']]=jmes['value']
        #print(" chan->"+jmes['chan'])
        #print(" value->"+jmes['value'])
        #lcd_state = "send_gpio"
        #if jmes['chan'] == 17 and jmes['value'] == 1:
        #    lstat += 1
        #if jmes['chan'] == 27 and jmes['value'] == 1:
        #    lstat -= 1

        #if lstat < 0:
        #    lstat = 2
        #if lstat > 2:
        #    lstat = 0
        #print(' Set new lstat ' + str(lstat) + ' lcd_state: ' + lcd_state[lstat])        
            
    if jmes['key'] == 'adc':
        jmv = jmes['vars']
        adc[0]=jmv['var1']
        adc[1]=jmv['var2']
        adc[2]=jmv['var3']
        adc[3]=jmv['var4']
        #if lcd_state == "send_gpio":
        #    lcd_state = "send_adc"


    #if jmes['key'] == 'lcd':
    #    mylcd.lcd_display_string(jmes["line1"],1)
    #    mylcd.lcd_display_string(jmes["line2"],2)
    # more callbacks, etc


def send_stuff(stuff):
    global tick
    global adc
    global switch
    if stuff == 0:
        count = 0
        while count < 5:
            tick +=1
            lcd_send(MQTT_PATH,str(tick) +" this is line 1", "this is line 2    ")
            time.sleep(1)
            count +=1
            
    if stuff == 1:
        count = 0
        while count < 5:
            lcd_send(MQTT_PATH,"IP Address:        ",get_ip_address('wlan0') + "     ")
            time.sleep(1)
            count += 1
    if stuff == 2:
        count = 0
        while count < 5:
            lcd_send(MQTT_PATH,get_cpu_date()+ "     ",get_cpu_time()+ "     ")
            time.sleep(1)
            count += 1
    if stuff == 3:
        count = 0
        while count < 5:
            os = get_cpu_temp()
            print "os = [" + os + "]"
            lcd_send(MQTT_PATH,"   --Cpu temp--   ",os[21:]+ "     ")
            time.sleep(1)
            count += 1
    if stuff == 4:
        count = 0
        while count < 5:
            os={}
            os[0] = 'xxxxxxxxxxxxxxxxxxxxxxxxxxx'
            os[1] = 'xxxxxxxxxxxxxxxxxxxxxxxxxxx'
            if len(switch) != 0:
                n = 0
                for key in switch.keys():
                    os[n] = "Gpio["+ str(key)+ "] val("+str(switch[key])+ ")"
                    n +=1
            lcd_send(MQTT_PATH,os[1],os[0])
            time.sleep(1)
            count += 1
    if stuff == 5:
        count = 0
        while count < 10:
            os={}
            os[0] = '0:' +adc[0]+' 1:' +adc[1] 
            os[1] = '2: ' +adc[2]+'3:' +adc[3]
            lcd_send(MQTT_PATH,os[0],os[1])
            time.sleep(0.5)
            count += 1
            
def send_lcd(stuff, ostat):
    global tick
    global adc
    global switch
    global lstat
    if stuff == 0:
        count = 0
        while count < 5 and lstat == ostat:
            tick +=1
            lcd_send(MQTT_PATH,str(tick%1000) +" cpump system   ", "version 0.1     ")
            time.sleep(1)
            count +=1
            
    if stuff == 1:
        count = 0
        while count < 5 and lstat == ostat:
            lcd_send(MQTT_PATH,"IP Address:        ",get_ip_address('wlan0') + "     ")
            time.sleep(1)
            count += 1
    elif stuff == 2:
        count = 0
        while count < 5 and lstat == ostat:
            lcd_send(MQTT_PATH,get_cpu_date()+ "     ",get_cpu_time()+ "     ")
            time.sleep(1)
            count += 1
    elif stuff == 3:
        count = 0
        while count < 5 and lstat == ostat:
            os = get_cpu_temp()
            print "os = [" + os + "]"
            lcd_send(MQTT_PATH,"   --Cpu temp--   ",os[21:]+ "     ")
            time.sleep(1)
            count += 1
    elif stuff == 4: 
        count = 0
        while count < 5 and lstat == ostat:
            os={}
            os[0] = 'xxxxxxxxxxxxxxxxxxxxxxxxxxx'
            os[1] = 'xxxxxxxxxxxxxxxxxxxxxxxxxxx'
            if len(switch) != 0:
                n = 0
                for key in switch.keys():
                    os[n] = "Gpio["+ str(key)+ "] val("+str(switch[key])+ ")"
                    n +=1
            lcd_send(MQTT_PATH,os[1],os[0])
            time.sleep(1)
            count += 1
    elif stuff == 5 and lstat == ostat:
        count = 0
        while count < 10 and lstat == ostat:
            os={}
            os[0] = '0:'  + adc[0] + ' 1:' + adc[1] 
            os[1] = '2: ' + adc[2] + ' 3:' + adc[3]
            lcd_send(MQTT_PATH, os[0], os[1])
            time.sleep(0.5)
            count += 1
          
if __name__ == "__main__":
    #global lcd_state
    #global lstat
    lstat = 0
    lcd_state[0] = "send_stuff"
    lcd_state[1] = "send_gpio"
    lcd_state[2] = "send_adc"
    print "connecting"
    #dir ('mqtt')
    client = mqtt.Client()
    #client
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_start()
    tick =0
    
    stuff = 0;
    while True:
        print "lstat = " + str(lstat)
        ostat = lstat
        if lcd_state[lstat] == "send_stuff":
            if stuff > 3:
                stuff = 0
            send_lcd(stuff, ostat)
            #send_stuff(stuff)
            stuff += 1
        elif lcd_state[lstat] == "send_gpio":
            count = 0
            while count < 5 and ostat == lstat:
                stuff = 4
                send_lcd(stuff, ostat)
                #send_stuff(stuff)
                count = count +1
            #lcd_state[lstat] = "send_stuff"    
        elif lcd_state[lstat] == "send_adc":
            count = 0
            while count < 1 and ostat == lstat:
                stuff = 5
                send_lcd(stuff, ostat)
                #send_stuff(stuff)
                count = count + 1
            #lcd_state = "send_stuff"    

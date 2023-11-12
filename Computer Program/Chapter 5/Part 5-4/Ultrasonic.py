import socket, struct
import RPi.GPIO as GPIO
import time

try:
    TCP_IP = ''
    TCP_PORT = 12345
    BUFFER_SIZE = 20
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.bind((TCP_IP,TCP_PORT))
    print("waiting for simulink to start")
    s.listen(1)
    conn, addr = s.accept()
    print("connection address: ", addr)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pinTrigger = 5
    pinEcho = 6
    
    GPIO.setup(pinTrigger, GPIO.OUT)
    GPIO.setup(pinEcho, GPIO.IN)
    
    GPIO.output(pinTrigger, GPIO.LOW)
    time.sleep(35)
    
    while True:
        GPIO.output(pinTrigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(pinTrigger, GPIO.LOW)
        
        pulseStartTime = time.time()
        pulseEndTime = time.time()
     
        while GPIO.input(pinEcho)==0:
            pulseStartTime = time.time()
        
        while GPIO.input(pinEcho)==1:
            pulseEndTime = time.time()
     
        pulseDuration = pulseEndTime - pulseStartTime
        distance = round(pulseDuration * 17150, 0)
     
        print("Distance: %.2f cm" % (distance))

        command = distance
        msg = struct.pack('<d', command)
        conn.send(msg)
        
        time.sleep(0.0000001)
        
except:
    GPIO.cleanup()
 
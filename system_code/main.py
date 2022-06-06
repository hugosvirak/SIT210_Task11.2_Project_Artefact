# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import pytesseract
import cv2 
import argparse 
import os 
import pytesseract
import requests
from PIL import Image
import smbus # a library to read or write from the system management bus
import time
import numpy as np
import RPi.GPIO as GPIO

# Setup

lightSensorAddress = 0x23
redLedAddress = 26 
yellowLedAddress = 13 
greenLedAddress = 6 
blueLedAddress = 5 

GPIO.setmode(GPIO.BCM)
GPIO.setup(redLedAddress, GPIO.OUT)
GPIO.setup(yellowLedAddress, GPIO.OUT)
GPIO.setup(greenLedAddress, GPIO.OUT)
GPIO.setup(blueLedAddress, GPIO.OUT)

bus = smbus.SMBus(1)

def getLightIntensity():
    # Read data from I2C interface
    try:
        data = bus.read_i2c_block_data(lightSensorAddress,0x20) # read light value from the address
        return (data[1] + (256 * data[0])) / 1.2 # convert from byte to number and return that value
    except:
        return None

#note: this function was copied from somewhere on stackoverflow
def rotateImage(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

#note: this function was copied from somewhere on stackoverflow
def applyBrightnessAndContrast(input_img, brightness = 0, contrast = 0):
    if brightness != 0:
        if brightness > 0:
            shadow = brightness
            highlight = 255
        else:
            shadow = 0
            highlight = 255 + brightness
        alpha_b = (highlight - shadow)/255
        gamma_b = shadow
        
        buf = cv2.addWeighted(input_img, alpha_b, input_img, 0, gamma_b)
    else:
        buf = input_img.copy()
    
    if contrast != 0:
        f = 131*(contrast + 127)/(127*(131-contrast))
        alpha_c = f
        gamma_c = 127*(1-f)
        
        buf = cv2.addWeighted(buf, alpha_c, buf, 0, gamma_c)
    return buf

def getReading():  
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    camera.close() 
    image = rawCapture.array
    image = cv2.rotate(image, cv2.cv2.ROTATE_180);
    image = image[122:198, 520:920]

    image = applyBrightnessAndContrast(image, 100, 120)
    image = rotateImage(image, -2)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    ret,image = cv2.threshold(image, 125, 255, cv2.THRESH_BINARY) 

    image = cv2.bitwise_not(image)

    #cv2.imwrite("the_image.png", image)
    reading = pytesseract.image_to_string(image, config='--psm 6 --oem 3 -c tessedit_char_whitelist=0123456789 -c page_separator=""')
    print(reading)
    reading_int = 0
    try:
        reading_int = int(reading)
    except ValueError: #TODO: more checks could be made, such as the number is equal or greater than previous numbers, to decrease chance of misreadings
        return None
    # allow the camera to shutdown
    time.sleep(0.1)
    return reading_int 


def sendSample(value): 
    try: 
        requests.get('http://192.168.0.43/new_sample', params={'value': value}, timeout=5)
    except:
        return False
    return True

def updateStatusLeds(value):
    GPIO.output(redLedAddress,GPIO.LOW)
    GPIO.output(yellowLedAddress,GPIO.LOW)
    GPIO.output(greenLedAddress,GPIO.LOW)
    GPIO.output(blueLedAddress,GPIO.LOW)

    if (value == "OK"): 
        GPIO.output(greenLedAddress,GPIO.HIGH)
    if (value == "NOT_OK"):
        GPIO.output(redLedAddress,GPIO.HIGH) 
    if (value == "WARNING"):
        GPIO.output(yellowLedAddress,GPIO.HIGH)
    if (value == "TOO_DARK"):
        GPIO.output(blueLedAddress,GPIO.HIGH)
try:
    while True:
        lightReading = getLightIntensity() 
        if lightReading is None: 
            updateStatusLeds("NOT_OK")
        elif (lightReading > 5):
            reading = getReading()
       	    if reading is None:
                 updateStatusLeds("WARNING")   
            else:
                if sendSample(reading) is False:
                    updateStatusLeds("NOT_OK")
                else:
                    updateStatusLeds("OK")
        else:
            updateStatusLeds("TOO_DARK") 
        print("waiting 2 seconds")
        time.sleep(2) # no need to read too often 
except KeyboardInterrupt:
    GPIO.cleanup() 

import serial
import time
import csv
import csv
import os
import sys
import numpy as np
import math
from vpython import *
import datetime

print("Skript zum auslesen und Visualisierung von Quaternion-Werte")
print("Bitte geben Sie den Port ein:")
port = input()
dataFromUART=serial.Serial(port,38400)
#time.sleep(1)

#Erstellen von Scene
scene.forward=vector(-1,-1,-1)
scene.width=500 
scene.height=500

ball = sphere(pos=vector(0,0,0),radius=7,opacity=0.3)
lamp = local_light(pos=vector(-3.5,3,4),color=color.yellow)
#Erstellen von Objekt(Zigbee Modull mit Sensor)
platteUnten=box(pos=vector(0,0,0),length=10,width=10,heigth=1,color=color.green,opacity=1)
Wand_a=box(pos=vector(3.5,1,4.5),length=3,width=1,heigth=0.5,color=color.black,opacity=0.6)
Wand_b=box(pos=vector(4.5,1,-1),length=1,width=1.5,heigth=0.5,color=color.black,opacity=0.6)
PlatteAufWand=box(pos=vector(3.5,2,1),length=3,width=8,heigth=1,color=color.green)
Wand_c=box(pos=vector(3.5,3,-2.5),length=3,width=1,heigth=0.5,color=color.black,opacity=0.6)
Wand_d=box(pos=vector(3.5,3,1),length=3,width=1,heigth=0.5,color=color.black,opacity=0.6)
sensor= box(pos=vector(3.5,4,-0.75),length=3,width=4.5,heigth=0.1,color=color.blue)
platteUntenZwei=box(pos=vector(0,2,-1),length=10,width=3.75,heigth=1,color=color.green)
Wand_e=box(pos=vector(-4,1,-2.25),length=2,width=1,heigth=0.5,color=color.black,opacity=1)
spitze=cone(pos=vector(0,4,-4),axis=vector(0,2,0),radius=1,opacity=0.7)
antenne = cylinder(pos=vector(0,0,-4),axis=vector(0,4,0), radius=1)
pointer1 = arrow(length= 8, axis=vector(1,0,0), shaftwidth=0.4,color=color.red)
pointer2 = arrow(length= 8,axis=vector(0,1,0), shaftwidth=0.4,color=color.blue)
pointer3 = arrow(length= 8,axis=vector(0,0,1), shaftwidth=0.4,color=color.yellow)
infos= label( pos=vec(0,13,0), text='Hello!' ) 
#Zusammenstellung von alle einzel Objekten
obj = compound([platteUnten, Wand_a,Wand_b,PlatteAufWand,Wand_d
                ,sensor,platteUntenZwei,Wand_e,spitze,Wand_c
                ,antenne,pointer1,pointer2,pointer3])#Statische Richungspfeile zur bessere Orientierung
frontArrow=arrow(length=10,shaftwidth=.3,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=10,shaftwidth=.3,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=10,shaftwidth=.3,color=color.orange,axis=vector(0,0,1))

#Erstelln/Vorbereitungen der CSV-Datei
f=open('Data.csv','w')
f.close()
time.sleep(0.5)

infoZeile = False
quaternionSensor=False
while (True):
    while (dataFromUART.inWaiting()==0):
        pass
    try:
        #Zeit:
        local_time = time.localtime()
        dt = datetime.datetime.now()
        #Konvertierung der sensorwerte.
        #Sensorwerte kommen in einer Reihen von Bytes an.
        
        dataPacket=dataFromUART.readline()
        dataPacket=str(dataPacket,'utf-8')
        print(dataPacket)
        splitPacket=dataPacket.split(",")
        
        
        var=(splitPacket[0][1])
        
        if(var!="E"):
            quaternionSensor=True
        else:
            quaternionSensor=False


        if quaternionSensor==True:
            #Separieren der Sensor-Werte
            q0=float(splitPacket[1])#w
            q1=float(splitPacket[2])#x
            q2=float(splitPacket[3])#y
            q3=float(splitPacket[4])#z
            q4=float(splitPacket[6])/1000#volt
            
            #Zusammensetzung der Werte und anschließend schreiben in der CSV-Datei
            
            arr=[q0,q1,q2,q3,q4,int(dt.microsecond/1000),local_time.tm_sec,local_time.tm_min,local_time.tm_hour,local_time.tm_mday,local_time.tm_mon,local_time.tm_year]
            neuerArray=",".join(str(x) for x in arr)
            
            infos.text="W: "+str(q0)+ " X: "+str(q1)+" Y: "+str(q2)+" Z: "+str(q3)+" Volt: "+str(q4)
            
            with open("data.csv", "a") as myfile:
                if(infoZeile==False):
                    title= 'Quaternion \n'
                    myfile.write(title)
                    data = 'w,x,y,z,volt,Millisekunden,Sekunde,Minute,Stunde,Tag,Monat,Jahr\n'
                    myfile.write(data)
                    infoZeile=True
                
                myfile.write("\n")
                myfile.write(neuerArray)
            
            #Berechnungen von Winkeln 
            roll=-math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
            pitch=math.asin(2*(q0*q2-q3*q1))
            yaw=-math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
            
            k=vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
            y=vector(0,1,0)
            s=cross(k,y)
            v=cross(s,k)
            vrot=v*cos(roll)+cross(k,v)*sin(roll)
            rate(50)
            #Die Winkeln an das Objekt anpassen
            obj.axis=k
            obj.up=vrot

        if quaternionSensor==False:
            for i in range(10):
                splitPacket[i] = splitPacket[i].strip()
            print(splitPacket)
            q0=(splitPacket[1])#acc
            q1=float(splitPacket[2])*1000.0#x_Acc
            q2=float(splitPacket[3])*1000#y_Acc
            q3=float(splitPacket[4])*1000#z_Acc

            q4=(splitPacket[5])#gyr
            q5=float(splitPacket[6])*1000#x_Gyr
            q6=float(splitPacket[7])*1000#y_Gyr
            q7=float(splitPacket[8])*1000#z_Gyr

            q8=(splitPacket[9])#volt
            q9=float(splitPacket[10])/1000#volt_Wert
            
            arr=[q1,q2,q3,q5,q6,q7,q9,int(dt.microsecond/1000),local_time.tm_sec,local_time.tm_min,local_time.tm_hour,local_time.tm_mday,local_time.tm_mon,local_time.tm_year]
            neuerArray=",".join(str(x) for x in arr)
            print(neuerArray)
            infos.text="Acc_x: "+str(q1)+ " Acc_y: "+str(q2)+" Acc_z: "+str(q3)+"\n"+" Gyr_x: "+str(q5)+" Gyr_y: "+str(q6)+" Gyr_z: "+str(q7)+"\n"+" Volt: "+str(q9)
            with open("data.csv", "a") as myfile:
                if(infoZeile==False):
                    title= 'Euler \n'
                    myfile.write(title)
                    data = 'Acc_x,Acc_y,Acc_z,Gyr_x,Gyr_y,Gyr_z,Volt,MilliSekunde,Sekunde,Minute,Stunde,Tag,Monat,Jahr\n'
                    myfile.write(data)
                    infoZeile=True
                myfile.write("\n")
                myfile.write(neuerArray)
            
            accelerometer_data=[q1,q2,q3]
            gyro_data=[q5,q6,q7]
            
            # Gyroskop-Daten
            gyro_data = np.array([q5,q6,q7])

            # Beschleunigungs-Daten
            accelerometer_data = np.array([q1, q2, q3])

            orientation_angles = np.zeros(3)
            for i in range(3):
                orientation_angles[i] = orientation_angles[i] + gyro_data[i] * dt.microsecond/100
            #Convert the accelerometer data to angles
            acceleration_angles = np.zeros(3)
            acceleration_angles[0] = np.arctan2(accelerometer_data[1], accelerometer_data[2])
            acceleration_angles[1] = np.arctan2(-accelerometer_data[0], np.sqrt(accelerometer_data[1]**2 + accelerometer_data[2]**2))
            
            #Combine the orientation angles and accelerometer angles using a complementary filter
            #Der Wert von alpha bestimmt das Verhältnis zwischen den Integrationsergebnissen
            #und den Beschleunigungsmesswerten. Ein höherer Wert von alpha bedeutet, dass die Integrationsergebnisse stärker gewichtet werden.
            alpha = 0.98
            for i in range(3):
                orientation_angles[i] = alpha * orientation_angles[i] + (1.0 - alpha) * acceleration_angles[i]
            rate(50)
            #Die Winkeln an das Objekt anpassen
            vec=vector(orientation_angles[0],orientation_angles[1],orientation_angles[2])
            #print(vec)
            obj.axis=vec
            #obj.up=orientation_angles
            #print(neuerArray)              

    except Exception as e: print(e)
        

 
    
    
    







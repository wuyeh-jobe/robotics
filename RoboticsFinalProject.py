#!/usr/bin/env python3
#Import Priority Queue
import queue as Q

#Import time
import time
from threading import Thread

#import copy
import copy

#Find the heuristic value of the grids
#The vaalue 100 represent obstacles
world_map = [[0, 100, 0,0,0,0],
             [0, 100, 0,0,0,0],
             [0, 100, 100, 100,0,0],
               [0, 0, 0,0,0,0]]


#This is the goal position on the map
GOAL = (1,2)
#The star cell is in the moveRobot function


#A function to find the estimate of distances from the gaol.
#return a map with heuristics calculated
def heuristic(map,START):
    queue = [(START[0],START[1])]

    while(len(queue)!=0):
        #note 1: Conditions to check if the neighbors being looked for are existing
        if(0 <= queue[0][0] <len(map) and 0<= queue[0][1]-1<len(map[0])):
            #Note 2: Checks if the the paths has not been traversed
            if(map[queue[0][0]][queue[0][1]-1]==0 and not(START[0]==queue[0][0] and queue[0][1]-1==START[1])):
                #print(map[queue[0][0]][queue[0][1]])
                map[queue[0][0]][queue[0][1]-1] = map[queue[0][0]][queue[0][1]] + 1
                queue.append((queue[0][0],queue[0][1]-1))
                #print("Left")
        #See note 1
        if(0 <= queue[0][0] <len(map) and 0<= queue[0][1]+1<len(map[0])):
            #See note 2
            if(map[queue[0][0]][queue[0][1]+1] ==0 and not(START[0]==queue[0][0] and queue[0][1]+1==START[1])):
                map[queue[0][0]][queue[0][1]+1] = map[queue[0][0]][queue[0][1]]+1
                queue.append((queue[0][0],queue[0][1]+1))
                #print("Right")
        #See note 1
        if(0 <= queue[0][0]-1 <len(map) and 0<= queue[0][1] <len(map[0])):
            #See note 2
            if(map[queue[0][0]-1][queue[0][1]] ==0 and not(START[0]==queue[0][0]-1 and queue[0][1]==START[1])):
                map[queue[0][0]-1][queue[0][1]] = map[queue[0][0]][queue[0][1]]+1
                queue.append((queue[0][0]-1,queue[0][1]))
                #print("Top")
        #See note 1
        if(0 <= queue[0][0]+1 <len(map) and 0<= queue[0][1]<len(map[0])):
            #See note 2
            if(map[queue[0][0]+1][queue[0][1]] == 0 and not(START[0]==queue[0][0]+1 and queue[0][1]==START[1])):
                map[queue[0][0]+1][queue[0][1]] = map[queue[0][0]][queue[0][1]]+1
                queue.append((queue[0][0]+1,queue[0][1]))
                #print("Bottom")
        #See note 1
        if(0 <= queue[0][0]-1 <len(map) and 0<= queue[0][1]+1<len(map[0])):
            #See note 2
            if(map[queue[0][0]-1][queue[0][1]+1] == 0 and not(START[0]==queue[0][0]-1 and queue[0][1]+1==START[1])):
                 if(map[queue[0][0]-1][queue[0][1]] == 100 and map[queue[0][0]][queue[0][1]+1] == 100):
                    print("no")
                 else: 
                    map[queue[0][0]-1][queue[0][1]+1] = map[queue[0][0]][queue[0][1]]+1
                    queue.append((queue[0][0]-1,queue[0][1]+1))
                    #print("Top_Right")
        #See note 1
        if(0 <= queue[0][0]-1 <len(map) and 0<= queue[0][1]-1<len(map[0])):
            #See note 2
            if(map[queue[0][0]-1][queue[0][1]-1]==0 and not(START[0]==queue[0][0]-1 and queue[0][1]-1==START[1])):
                  if(map[queue[0][0]-1][queue[0][1]] ==100 and map[queue[0][0]][queue[0][1]-1]==100):
                    print("no")
                  else:
                     map[queue[0][0]-1][queue[0][1]-1] = map[queue[0][0]][queue[0][1]]+1
                     queue.append((queue[0][0]-1,queue[0][1]-1))
                     #print("Top_Left")
        #See note 1
        if(0 <= queue[0][0]+1 <len(map) and 0<= queue[0][1]+1<len(map[0])):
            #See note 2
            if(map[queue[0][0]+1][queue[0][1]+1] ==0 and not(START[0]==queue[0][0]+1 and queue[0][1]+1==START[1])):
                 if(map[queue[0][0]+1][queue[0][1]]==100 and map[queue[0][0]][queue[0][1]+1] == 100):
                    print("no")
                 else:
                     map[queue[0][0]+1][queue[0][1]+1] = map[queue[0][0]][queue[0][1]]+1
                     queue.append((queue[0][0]+1,queue[0][1]+1))
                     #print("Bottom_Right")
        #See note 1
        if(0 <= queue[0][0]+1 <len(map) and 0<= queue[0][1]-1<len(map[0])):
            #See note 2
            if(map[queue[0][0]+1][queue[0][1]-1] ==0 and not(START[0]==queue[0][0]+1 and queue[0][1]-1==START[1])):
                 if(map[queue[0][0]+1][queue[0][1]] ==100 and map[queue[0][0]][queue[0][1]-1]==100):
                    print("no")
                 else:
                     map[queue[0][0]+1][queue[0][1]-1] = map[queue[0][0]][queue[0][1]]+1
                     queue.append((queue[0][0]+1,queue[0][1]-1))
                     #print("Bottom_Left")
        del queue[0]
    return map


#This function calculates the shortest path using back propagation
#First, the goal cell is inserted into the priority queue.
#Then the neigbors of the priority cell are also put into the queue
#This would be iterated untill it reaches the start.
#By the time the start is reached, the goal would have been calculated

def calulateSP(ahc,START):
    #Instantiate priority queue
    q = Q.PriorityQueue()
    
    q.put((ahc[GOAL[0]][GOAL[1]],time.time(),GOAL))
    #Path calculated
    path = []
    get = q.get()
    while ahc[get[2][0]][get[2][1]] !=0:
        #This is to ensure that priority is given to top cells when the robot is
        #moving upwards and right, from the start to the goal
        if GOAL[1] > START[1] and GOAL[0]<START[0]:
            #note 1: Conditions to check if the neighbors being looked for are existing
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("LN Exist")
                q.put((ahc[get[2][0]][get[2][1]-1],time.time(),(get[2][0],get[2][1]-1)))
                time.sleep(0.1)
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("RN Exist")
                q.put((ahc[get[2][0]][get[2][1]+1],time.time(), (get[2][0],get[2][1]+1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]<len(ahc[0])):
                #print("BN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]],time.time(),(get[2][0]+1,get[2][1])))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("BLN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]-1],time.time(),(get[2][0]+1,get[2][1]-1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1] <len(ahc[0])):
                #print("TN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]],time.time(),(get[2][0]-1,get[2][1])))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("TRN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]+1],time.time(),(get[2][0]-1,get[2][1]+1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("TLN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]-1],time.time(),(get[2][0]-1,get[2][1]-1)))
            
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("BRN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]+1],time.time(),(get[2][0]+1,get[2][1]+1)))
                
        #This is to ensure that priority is given to down cells when the robot is
        #moving down and right, from the start to the goal  
        elif GOAL[1] > START[1] and GOAL[0]>START[0]:
            #note 1: Conditions to check if the neighbors being looked for are existing
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("LN Exist")
                q.put((ahc[get[2][0]][get[2][1]-1],time.time(),(get[2][0],get[2][1]-1)))
                time.sleep(0.1)
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("RN Exist")
                q.put((ahc[get[2][0]][get[2][1]+1],time.time(), (get[2][0],get[2][1]+1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("TLN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]-1],time.time(),(get[2][0]-1,get[2][1]-1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1] <len(ahc[0])):
                #print("TN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]],time.time(),(get[2][0]-1,get[2][1])))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]<len(ahc[0])):
                #print("BN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]],time.time(),(get[2][0]+1,get[2][1])))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("TRN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]+1],time.time(),(get[2][0]-1,get[2][1]+1)))
            
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("BRN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]+1],time.time(),(get[2][0]+1,get[2][1]+1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("BLN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]-1],time.time(),(get[2][0]+1,get[2][1]-1)))
                
        #This is to ensure that priority is given to down and cost effective cells when the robot is
        #moving down and left, from the start to the goal
        elif GOAL[1]<START[1] and GOAL[0]>START[0]:
            #note 1: Conditions to check if the neighbors being looked for are existing
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("RN Exist")
                q.put((ahc[get[2][0]][get[2][1]+1],time.time(), (get[2][0],get[2][1]+1)))
                time.sleep(0.1)
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("LN Exist")
                q.put((ahc[get[2][0]][get[2][1]-1],time.time(),(get[2][0],get[2][1]-1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("TRN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]+1],time.time(),(get[2][0]-1,get[2][1]+1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1] <len(ahc[0])):
                #print("TN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]],time.time(),(get[2][0]-1,get[2][1])))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]<len(ahc[0])):
                #print("BN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]],time.time(),(get[2][0]+1,get[2][1])))
            
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("TLN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]-1],time.time(),(get[2][0]-1,get[2][1]-1)))
            
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("BRN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]+1],time.time(),(get[2][0]+1,get[2][1]+1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("BLN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]-1],time.time(),(get[2][0]+1,get[2][1]-1)))
                
        #This is to ensure that priority is given to top and cost effective cells when the robot is
        #moving up and left, from the start to the goal
        elif GOAL[1]<START[1] and GOAL[0]<START[0]:
            #note 1: Conditions to check if the neighbors being looked for are existing
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("RN Exist")
                q.put((ahc[get[2][0]][get[2][1]+1],time.time(), (get[2][0],get[2][1]+1)))
                time.sleep(0.1)
            
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("LN Exist")
                q.put((ahc[get[2][0]][get[2][1]-1],time.time(),(get[2][0],get[2][1]-1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("BRN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]+1],time.time(),(get[2][0]+1,get[2][1]+1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]<len(ahc[0])):
                #print("BN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]],time.time(),(get[2][0]+1,get[2][1])))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("TRN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]+1],time.time(),(get[2][0]-1,get[2][1]+1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1] <len(ahc[0])):
                #print("TN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]],time.time(),(get[2][0]-1,get[2][1])))
            
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("TLN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]-1],time.time(),(get[2][0]-1,get[2][1]-1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("BLN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]-1],time.time(),(get[2][0]+1,get[2][1]-1)))
                
        #This takes care of the rest of the situations
        else:
            #note 1: Conditions to check if the neighbors being looked for are existing
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("RN Exist")
                q.put((ahc[get[2][0]][get[2][1]+1],time.time(), (get[2][0],get[2][1]+1)))
            if(0 <= get[2][0] <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("LN Exist")
                q.put((ahc[get[2][0]][get[2][1]-1],time.time(),(get[2][0],get[2][1]-1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1] <len(ahc[0])):
                #print("TN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]],time.time(),(get[2][0]-1,get[2][1])))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]<len(ahc[0])):
                #print("BN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]],time.time(),(get[2][0]+1,get[2][1])))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("TRN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]+1],time.time(),(get[2][0]-1,get[2][1]+1)))
            if(0 <= get[2][0]-1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("TLN Exist")
                q.put((ahc[get[2][0]-1][get[2][1]-1],time.time(),(get[2][0]-1,get[2][1]-1)))
            
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]+1<len(ahc[0])):
                #print("BRN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]+1],time.time(),(get[2][0]+1,get[2][1]+1)))
            if(0 <= get[2][0]+1 <len(ahc) and 0<= get[2][1]-1<len(ahc[0])):
                #print("BLN Exist")
                q.put((ahc[get[2][0]+1][get[2][1]-1],time.time(),(get[2][0]+1,get[2][1]-1)))
            
        path.append((get[0],get[2]))
        get = q.get()
    path.append((get[0],get[2]))
    return path   
            

import ev3dev.ev3 as ev3
from ev3dev.ev3 import *

# Connect ultrasonic and touch sensors to any sensor port
# and check they are connected.
us = UltrasonicSensor() 
assert us.connected, "Connect a single US sensor to any sensor port"
ts = TouchSensor()
assert ts.connected, "Connect a touch sensor to any port"

###This functions turn the Robot Left,Right Straight
circ = 8

def straight(distance, speed):
  rightM.run_to_rel_pos(position_sp=(360*distance)/circ, speed_sp=speed, stop_action="brake")
  leftM.run_to_rel_pos(position_sp=(360*distance)/circ, speed_sp=speed, stop_action="brake")
  rightM.wait_while('running')
  leftM.wait_while('running')

def turnLeft(degrees, speed):
  rightM.run_to_rel_pos(position_sp=4*degrees, speed_sp=speed, stop_action="brake")
  leftM.run_to_rel_pos(position_sp=-4*degrees, speed_sp=speed, stop_action="brake")
  #rightM.wait_while('running')
  rightM.wait_while('running')
  leftM.wait_while('running')

def turnRight(degrees, speed):
  leftM.run_to_rel_pos(position_sp=4*degrees, speed_sp=speed, stop_action="brake")
  rightM.run_to_rel_pos(position_sp=-4*degrees, speed_sp=speed, stop_action="brake")
  rightM.wait_while('running')
  leftM.wait_while('running')

###################################################################

rightM = ev3.LargeMotor('outC')
leftM = ev3.LargeMotor('outB')

#This function is resposible for moving the robot in the right direction when
#the shortest path is generated
#It has a lot of conditions because movement can be in 8 ways:
# 1. Left 2. Right 3. Up  4. Down  5. Top Left 6. Top Right 7. Botton Left 8. Bottom Right

def moveRobot(): 
 START = (3,1)
 #Calculated Heuristics
 x = heuristic(world_map, START)
 for i in range(len(x)):
     print(x[i])
 sPath = calulateSP(x,START)
 print(sPath)
 pLength = len(sPath)-1
 ot = "up"
 
 while pLength>=0:
  if ot == "up":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("left")
          turnLeft(42,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("right")
          turnRight(42,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("Stra")
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turn180")
          turnRight(83,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn45OLef")
          #TLeft
          turnLeft(17,500)
          straight(29,500)
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn45oRig")
          #TRight
          turnRight(17,500)
          straight(29,500)
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn135oLef")
          #Bleft
          turnLeft(62,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn135oRight")
          #BRight
          turnRight(62,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "down":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("right")
          turnRight(42,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("left")
          turnLeft(42,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turn180")
          turnRight(83,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("Straight")
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn135Oright")
          #TL
          turnRight(62,500)
          straight(29,500)
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn135olef")
          #TR
          turnLeft(62,500)
          straight(29,500)
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn45oRig")
          #BL
          turnRight(17,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn45oLef")
          #BR
          turnLeft(17,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "left":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("stra")
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("turn180")
          turnRight(82,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("right")
          turnRight(42,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("left")
          turnLeft(42,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn45Oright")
          #TL
          turnRight(17,500)
          straight(29,500)
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn135oright")
          #TR
          turnRight(62,500)
          straight(29,500)
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn45oLef")
          #BL
          turnLeft(17,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn135oleft")
          turnLeft(62,500)
          straight(29,500)
          #BR
          ot = "bRight"
  elif ot == "right":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("turn180")
          turnRight(83,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("straigh")
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("left")
          turnLeft(42,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("right")
          turnRight(42,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn135Olef")
          turnLeft(62,500)
          straight(29,500)
          #TL
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn45olef")
          turnLeft(17,500)
          straight(29,500)
          #TR
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn135o righ")
          #BL
          turnRight(62,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn45oRight")
          #BR
          turnRight(17,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "tLeft":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("turnLeft 45")
          turnLeft(17,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("turnRight135")
          turnRight(62,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("left")
          turnRight(17,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turn135 left")
          turnLeft(62,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("straight")
          straight(29,500)
          #TL
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn90oright")
          turnRight(42,500)
          straight(29,500)
          #TR
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn90o left")
          #BL
          turnLeft(42,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn180")
          #BR
          turnRight(83,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "tRight":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("turnLeft 135")
          turnLeft(62,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("turnRight45")
          turnRight(17,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turnLeft45")
          turnLeft(17,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turn135 right")
          turnRight(62,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn90 left")
          turnLeft(42,500)
          straight(29,500)
          #TL
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("straight")
          straight(29,500)
          #TR
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn180")
          #BL
          turnLeft(83,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn90 right")
          #BR
          turnRight(42,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "bLeft":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("turnRight 45")
          turnRight(17,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("turnLeft 135")
          turnLeft(62,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turnRight135")
          turnRight(62,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turnLeft 45")
          turnLeft(17,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turnRight 90")
          turnRight(42,500)
          straight(29,500)
          #TL
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn 180")
          turnRight(83,500)
          straight(29,500)
          #TR
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("straight")
          #BL
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn90 Left")
          #BR
          turnLeft(42,500)
          straight(29,500)
          ot = "bRight"
  elif ot == "bRight":
      if sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == 1:
          print("turnRight 135")
          turnRight(62,500)
          straight(20,500)
          ot = "left"
      elif sPath[pLength][1][0]== sPath[pLength-1][1][0] and sPath[pLength][1][1]- sPath[pLength-1][1][1] == -1:
          print("turnLeft 45")
          turnLeft(17,500)
          straight(20,500)
          ot = "right"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turnLeft135")
          turnLeft(62,500)
          straight(20,500)
          ot = "up"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] == sPath[pLength-1][1][1]:
          print("turn45 Right")
          turnRight(17,500)
          straight(20,500)
          ot = "down"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn180")
          turnRight(83,500)
          straight(29,500)
          #TL
          ot = "tLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == 1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("turn9o Left")
          turnLeft(42,500)
          straight(29,500)
          #TR
          ot = "tRight"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==1:
          print("turn 90 right")
          #BL
          turnRight(42,500)
          straight(29,500)
          ot = "bLeft"
      elif sPath[pLength][1][0]- sPath[pLength-1][1][0] == -1 and sPath[pLength][1][1] - sPath[pLength-1][1][1] ==-1:
          print("straight")
          #BR
          straight(29,500)
          ot = "bRight"
  pLength = pLength-1

moveRobot()

    
    

    

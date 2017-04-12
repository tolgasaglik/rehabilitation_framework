#!/usr/bin/env python

import roslib; roslib.load_manifest('reha_game')
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


from random import randint


DEBUG = False
#                 |y=-0.1
#        D        |      A
#x=-0.1           |             x=0.1
#------------------------------------
#                 |
#       C         |      B
#                 |y=0.1



# Node example class.
class NodeGame():
    
    def game_start_callback(self, event):
        #print 'Timer called at ' + str(event.current_real)
        print 'Starting the game: ' + str(event.data)
        self.game_mode = int (event.data) 
        
        if( self.game_mode == 0 ):
           self.pub.publish("Robot in Waiting mode")
        elif( self.game_mode == 1 ):
            self.pub.publish("Lateral training selected")
        elif( self.game_mode == 2 ):
            self.pub.publish("Frontal training selected")
            
        self.movementGo == False    
        self.loop
        
    
    def timer_callback(self, event):
        #print 'Timer called at ' + str(event.current_real)
        print 'Status of the game >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><: ' + str(self.game_status)
    
    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)
        rospy.loginfo("====")
        self.game_status = 1
        if (DEBUG):
            rospy.loginfo(rospy.get_caller_id() + "x %s", data.pose.position.x)
            rospy.loginfo(rospy.get_caller_id() + "y %s", data.pose.position.y)
        
        if (DEBUG):
            if (data.pose.position.x > 0.0) and (data.pose.position.y < 0.0):
                rospy.loginfo("Frame %s", 'A')
            elif (data.pose.position.x > 0.0) and (data.pose.position.y > 0.0):
                rospy.loginfo("Frame %s", 'B')
            elif (data.pose.position.x < 0.0) and (data.pose.position.y > 0.0):
                rospy.loginfo("Frame %s", 'C')
            elif (data.pose.position.x < 0.0) and (data.pose.position.y < 0.0):
                rospy.loginfo("Frame %s", 'D')
            
        positionX = data.pose.position.x * 100
        positiony = data.pose.position.y * 100 
        positionZ = data.pose.position.z * 100 
        
        if(self.game_mode == 0):
            print 'Waiting to establish the game type<: '
        
        elif(self.game_mode == 1):
            gameXPosition = 0
            correctDistance = False
            
            
            if (positionZ < 110) and (positionZ > 90):
                gameXPosition = int((positionX + 40) / 4)
                correctDistance = True
            else:
                correctDistance = False
            
            if (True):
                print "Position: " + str(positionX)
                print "Game position: " + str(gameXPosition)
                print "Distance: " + str(data.pose.position.z * 100 )
            
            if ( gameXPosition < self.last_moment-1 ):
                #self.last_moment = moment
                self.moment_status = -1
            elif ( gameXPosition > self.last_moment+1):
                #self.last_moment = moment
                self.moment_status = 1

            self.last_moment = gameXPosition            
            
            if (correctDistance):
                print "Status - gameXPosition [" + str(gameXPosition) + "], self.last_moment ["+ str(self.last_moment)
                print "Status - moment_status [" + str(self.moment_status) + "], self.last_moment_status ["+ str(self.last_moment_status)    
                print "> " + str(self.loop)
                print "> " + str(self.loop % 2)
                now = rospy.get_rostime()
                print now
                
            #if ( self.moment_status != self.last_moment_status ):
            if ( correctDistance ) and ( gameXPosition <= 2 ) and ( self.movementGo == False):
                #last_moment_status = moment_status
                print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> :" + str(self.loop)
                #self.loop += 1
                self.movementGo = True
                #rospy.sleep(1)
                #self.pub.publish("Yes")
                #if (self.loop % 2):
                    #self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                
                #os.system('rosrun sound_play say.py "yes"')
                #subprocess.call(["rosrun", "sound_play say.py \"yes\""])
                if ( self.loop != 5 ):
                    self.pub.publish(self.numbers[self.loop])
                    self.loop += 1
                elif ( self.loop == 5 ):
                    self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                    self.loop = 0
            if ( correctDistance ) and ( gameXPosition >=9 ) and ( self.movementGo == True):
                #last_moment_status = moment_status
                print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> :" + str(self.loop)
                #self.loop += 1
                self.movementGo = False
                #rospy.sleep(1)
                #self.pub.publish("Yes")
                #if (self.loop % 2):
                    #self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                
                #os.system('rosrun sound_play say.py "yes"')
                #subprocess.call(["rosrun", "sound_play say.py \"yes\""])
                
                #
                #To count each movement
                #if ( self.loop != 5 ):
                    #self.pub.publish(self.numbers[self.loop])
                    #self.loop += 1
                #elif ( self.loop == 5 ):
                    #self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                    #self.loop = 0
                    
            self.last_moment_status = self.moment_status
            self.game_status = 0
            
        ##This is the part for the second game, the roll in the table.    
        elif (self.game_mode == 2):
            
            positionX = int ( data.pose.position.x * 100 )
            positiony = int ( data.pose.position.y * 100 )
            positionZ = int ( data.pose.position.z * 100 )
            
            if(True):
                print "Distance: " + str(data.pose.position.z * 100 )
                #print "X: " + str(data.pose.position.x * 100 ) + ". Y: " + str(data.pose.position.y * 100 )
                
            if ( positionZ > 110 ) and ( self.movementGo == False):
                print "===========================================================================" 
                print "Up :" + str(self.loop)
                print "===========================================================================" 
                self.movementGo = True
                if ( self.loop != 5 ) :
                    self.pub.publish(self.numbers[self.loop])
                    self.loop += 1
                elif ( self.loop == 5 ):
                    self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                    self.loop = 0
            elif ( positionZ < 80 ) and ( self.movementGo == True):
                
                print "===========================================================================" 
                print "down :" + str(self.loop)
                print "===========================================================================" 
                self.movementGo = False
                #if ( self.loop != 5 ) :
                    #self.pub.publish(self.numbers[self.loop])
                #self.loop += 1
                #elif ( self.loop == 5 ):
                    #self.pub.publish(self.enc_sentences[randint(0,self.total_enc_sentences-1)])
                    #self.loop = 0
            
            

    def __init__(self):
        self.game_mode = 0
        self.game_status = False
        self.last_moment = 0        
        self.moment_status = 0  
        self.last_moment_status = 0
        self.loop = 0
        self.enc_sentences = []
        self.numbers = ["one", "two", "three", "four", "five" ]
        self.movementGo = False
        
        
        file = open("/home/leandrogil/Documents/catkin_ws/src/reha_game/config/encouragement_sentences.txt", "r") 
        for line in file: 
            self.enc_sentences.append(line.strip('\n')) 
            print line 
        
        self.total_enc_sentences  = len(self.enc_sentences)
        print self.total_enc_sentences
        print self.enc_sentences[0]
        
        rospy.Timer(rospy.Duration(3), self.timer_callback)

         # Create the sound client object
        #self.soundhandle = SoundClient()
        #rospy.sleep(1)
        #self.soundhandle.stopAll()
        
        #self.voice = "voice_kal_diphone"
        #self.soundhandle.say("Hello", self.voice)
        #rospy.sleep(1)
        
        
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous node is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        #rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/visualization_marker", Marker, self.callback)
        rospy.Subscriber("/rehabilitation_task/mode", String, self.game_start_callback)
        
        self.pub = rospy.Publisher('/robot/voice', String, queue_size=10)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        print "exit"

    
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('game', anonymous=True)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = NodeGame()
    except rospy.ROSInterruptException: pass
    

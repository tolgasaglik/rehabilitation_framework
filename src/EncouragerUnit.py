import rospy,rospkg
from std_msgs.msg import String
import random


class EncouragerUnit(object):
    ENC_SENT_FILELOC = "/config/encouragement_sentences.txt"
    _happy_emotions_list = ["happy", "neutral", "showing_smile", "surprise", "breathing_exercise", "breathing_exercise_nose", "smile", "happy_blinking", "calming_down"]

    @property
    def repetitions_limit(self):
        return self._repetitions_limit

    @property
    def repetitions_arr(self):
        return self._repetitions_arr

    @property
    def spawn_ros_nodes(self):
        return self._spawn_ros_nodes
    @spawn_ros_nodes.setter
    def spawn_ros_nodes(self, spawn_ros_nodes):
        if type(spawn_ros_nodes) is bool:
            self._spawn_ros_nodes = spawn_ros_nodes
        else:
            raise ValueError("Argument spawn_ros_nodes is not boolean!")

    def __init__(self, number_of_blocks=1, repetitions_limit=10, quantitative_frequency=0, qualitative_frequency=0, emotional_feedbacks=[]):
        self._voice_pub = rospy.Publisher('/robot/voice', String, queue_size=5)
        self._face_pub = rospy.Publisher('/qt_face/setEmotion', String, queue_size=1)
        self._gesture_pub = rospy.Publisher('/robotMovementfromFile', String, queue_size=1)
        self.load_sentences()
        self._repetitions_limit = repetitions_limit
        self._repetitions_arr = [0] * number_of_blocks
        self._emotional_feedback_list = emotional_feedbacks
        self._quantitative_frequency = quantitative_frequency
        self._qualitative_frequency = qualitative_frequency
        random.seed()
        
    def load_sentences(self):
        # retrieve path to reha_game ROS package and load sentences file
        rospack = rospkg.RosPack()
        reha_game_pkg_path = rospack.get_path('rehabilitation_framework')
        sentences_file = open(reha_game_pkg_path + self.ENC_SENT_FILELOC, "r")
        self._sentences = sentences_file.readlines()
        sentences_file.close()

    def inc_repetitions_counter(self, index):
        self._repetitions_arr[index] += 1

        # process emotional feedbacks (= show an emotion on the QT robot face)
        for efb in self._emotional_feedback_list:
            # is feedback fixed? (yes/no)
            if efb[0] == True and self._repetitions_arr[index] == efb[1] or efb[0] == False and self._repetitions_arr[index] % efb[1] == 0 and self._repetitions_arr[index] > 0:
                if efb[2] == "random emotion":
                    self._face_pub.publish(self._happy_emotions_list[random.randint(0,len(self._happy_emotions_list)-1)])
                else:
                    self._face_pub.publish(efb[2])
                if efb[3] == True:
                    self._gesture_pub.publish("testTo5")

        # process quantitative feedback (= tell patient how many repetitions he has done so far)
        if self._quantitative_frequency > 0 and self._repetitions_arr[index] in range(0,self._repetitions_limit-1) and self._repetitions_arr[index] % self._quantitative_frequency == 0:
            self._voice_pub.publish(str(self._repetitions_arr[index]))
            print "Current number of repetitions done: " + str(self._repetitions_arr[index])

        # process qualitative feedback (= tell patient some randomly chosen motivational sentence)
        if self._qualitative_frequency > 0 and self._repetitions_arr[index] in range(0,self._repetitions_limit-1) and self._repetitions_arr[index] % self._qualitative_frequency == 0:
            self._voice_pub.publish(self._sentences[random.randint(0,len(self._sentences)-1)])

    def say(self, sentence):
        self._voice_pub.publish(sentence)
        print "The robot says: \"" + sentence + "\""

    def show_emotion(self, emotion):
	if emotion not in self._happy_emotions_list:
	    raise Exception("Emotion not recognized!")
        else:
            self._face_pub.publish(emotion)
            print("The robot is showing the following emotion: " + emotion + "!")

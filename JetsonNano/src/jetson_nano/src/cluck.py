from pydub import AudioSegment 
from pydub.playback import play
import rospy
from customized_msgs.msg import SensorData

class AudioPlayer: 
    def __init__(self):
        # params
        self.open = False
        self.cluck = AudioSegment.from_file(file="sound.wav", format="wav") # constant
        self.rate = rospy.Rate(10)
        # subscriber
        rospy.Subscriber("sensor_data", SensorData, self.callback)

    # check to play sound
    def callback(self, msg):
        if msg.grasper_angle > .9 and self.open == True: 
            play(self.cluck)
            self.open == False
        elif msg.grasper_angle <= .9:
            self.open == True

if __name__ == '__main__':
    rospy.init_node("cluck_audio")
    audio_player = AudioPlayer()
#!/usr/bin/env python

import argparse
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio


class HuskyVoiceController(object):

    def __init__(self, model, lexicon, kwlist):
        #rospy node initialization for publishing
        rospy.init_node('husky_voice_dictation', anonymous=True) # for publishing later
        rospy.on_shutdown(self.shutdown) # tell ros what to do when Ctrl + C is pressed
        pubInt = rospy.Publisher('voice_command_int', Int8, queue_size=10) # int of command publishing, possibly change queue size to 0 for debug/natural response
        pubStr = rospy.Publisher('voice_command_str', String, queue_size=10) # str of command publishing (more of a log at the moment)

        # initialize pocketsphinx
        config = Decoder.default_config() # set up a decoder to listen, pass command line arguments
        config.set_string('-hmm', model)
        config.set_string('-dict', lexicon)
        config.set_string('-kws', kwlist)

	   # start microphone audio stream
        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()
	
	   # audio decoder starts listening
        self.decoder = Decoder(config)
        self.decoder.start_utt()

        while not rospy.is_shutdown(): # continous loop to run while rospy is initialized
            buf = stream.read(1024) # check audio stream for voice commands
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_result()

    def parse_result(self):
        if self.decoder.hyp() != None:
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search\n")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()

            if (seg.word.find("forward") > -1):
                #pubInt.publish(2)
                print("Found Forward ****************************")

            if (seg.word.find("backward") > -1):
                #pubInt.publish(3)
                print("Found backward ***************************")
            
    def shutdown(self): # exectuted when Ctrl + C is pressed
        rospy.loginfo("Husky Voice Controller Shutdown")
        rospy.sleep(1)

if __name__ == '__main__':
    # command line arguments in case defaults don't suffice
    parser = argparse.ArgumentParser(
        description='Control ROS husky using pocketsphinx.')
    parser.add_argument('--model', type=str,
        default='/home/tyler/model/en_US',
        help='''acoustic model path (default: model/en_US)''')
    parser.add_argument('--lexicon', type=str,
        default='/home/tyler/voice_cmd.dic',
        help='''pronunciation dictionary (default: voice_cmd.dic)''')
    parser.add_argument('--kwlist', type=str,
        default='/home/tyler/voice_cmd.kwlist',
        help='''keyword list with thresholds (default: voice_cmd.kwlist)''')
    args = parser.parse_args() # run arg parser
    HuskyVoiceController(args.model, args.lexicon, args.kwlist) # run control function


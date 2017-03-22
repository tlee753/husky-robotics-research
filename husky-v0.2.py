#!/usr/bin/env python

import argparse
import rospy

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio


class HuskyVoiceController(object):
"""
Parameters:
	model: language model path
	lexicon: pronunciation dictionary
	kwlist: keyword list file
	pub: where to send commands (default: '')
"""
    def __init__(self, model, lexicon, kwlist, pub):
        #rospy initialization for publishing
        rospy.init_node('husky_voice_dictation') # for publishing later
        rospy.on_shutdown(self.shutdown)

        # initialize pocketsphinx
        config = Decoder.default_config()
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
            buf = stream.read(1024)
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
                print("FORWARD HAS BEEN FOUND PUBLISH ME")

            if (seg.word.find("backward") > -1):
                print("BACKWARD HAS BEEN FOUND PUBLISH ME")
            

    def shutdown(self): # exectuted when Ctrl + C is pressed
        rospy.loginfo("S")
        rospy.sleep(1)

if __name__ == '__main__':
    # command line arguments in case defaults don't suffice
    parser = argparse.ArgumentParser(
        description='Control ROS husky using pocketsphinx.')
    parser.add_argument('--model', type=str,
        default='model/en_US',
        help='''acoustic model path (default: model/en_US)''')
    parser.add_argument('--lexicon', type=str,
        default='voice_cmd.dic',
        help='''pronunciation dictionary (default: voice_cmd.dic)''')
    parser.add_argument('--kwlist', type=str,
        default='voice_cmd.kwlist',
        help='''keyword list with thresholds (default: voice_cmd.kwlist)''')
    parser.add_argument('--rospub', type=str,
        default='mobile_base/commands/velocity',
        help='''ROS publisher destination (default: mobile_base/commands/velocity)''')

    args = parser.parse_args() # run arg parser
    ASRControl(args.model, args.lexicon, args.kwlist, args.rospub) # run control function


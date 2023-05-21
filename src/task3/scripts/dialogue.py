#!/usr/bin/python3
import roslib

roslib.load_manifest("sound_play")
import rospy
import speech_recognition as sr
from task3.srv import FaceDialogueSrv, FaceDialogueSrvResponse
from sound_play.libsoundplay import SoundClient


MIC_IX = 2


class Dialogue:
    def __init__(self):
        rospy.init_node("dialogue_node", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.recognizer = sr.Recognizer()
        self.sound_client = SoundClient()

        self.dialogue_service = rospy.Service(
            "/dialogue_service", FaceDialogueSrv, self.handle_dialogue_service
        )
        rospy.sleep(1)

    def handle_dialogue_service(self, req):
        rospy.loginfo("Dialogue service called.")
        resp = self.dialogue()
        rospy.loginfo("Dialoge service finished.")

        if len(resp) != 2:
            return FaceDialogueSrvResponse(False, color1="", color2="")

        return FaceDialogueSrvResponse(True, color1=resp[0], color2=resp[1])

    def dialogue(self):
        response = self.check_for_useful_info(self.ask_for_robber())
        response.extend(self.check_for_useful_info(self.ask_for_robber()))

        if len(response) == 0:
            self.sound_client.say("Sorry, I didn't get any useful information.")
        else:
            self.sound_client.say(
                "Ok, I will look for the robber on the "
                + ", ".join(response)
                + " cylinder."
            )

        return response

    def check_for_useful_info(self, response):
        colors = []
        if "blue" in response.lower():
            colors.append("blue")
        if "red" in response.lower():
            colors.append("red")
        if "green" in response.lower():
            colors.append("green")
        if "yellow" in response.lower():
            colors.append("yellow")
        if "blue" in response.lower():
            colors.append("blue")

        return colors

    def ask_for_robber(self):
        self.sound_client.say("Hey, do you know where the robber is hiding?")
        rospy.sleep(2)
        response = self.recognize_speech_from_mic()
        if response["success"] != True:
            self.sound_client.say("I didn't catch that. Could you please repeat?")
            rospy.sleep(2)
            response = self.recognize_speech_from_mic()
            if response["success"] != True:
                self.sound_client.say("Falling back to keyboard input.")
                response["transcription"] = self.enter_text()

        return response["transcription"]

    def enter_text(self):
        user_input = input("Hey, do you know where the robber is hiding?  ")
        return user_input

    def recognize_speech_from_mic(self):
        with sr.Microphone(device_index=MIC_IX) as source:
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Say something!")
            audio = self.recognizer.listen(source)

        response = {"success": True, "error": None, "transcription": None}

        try:
            response["transcription"] = self.recognizer.recognize_google(audio)
        except sr.RequestError:
            response["success"] = False
            response["error"] = "API unavailable"
        except sr.UnknownValueError:
            response["error"] = "Unable to recognize speech"

        rospy.loginfo(response)
        return response

    def shutdown(self):
        rospy.loginfo("Dialogue node shutting down.")

    def run(self):
        rospy.spin()


def main():
    dialogue = Dialogue()
    rospy.loginfo("Dialogue node started.")

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

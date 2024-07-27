#!/usr/bin/env python3
import speech_recognition as sr
import pyaudio
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from threading import Thread

# Define a global variable to indicate collision status
collision_detected = False

def get_default_input_device_index():
    audio = pyaudio.PyAudio()
    default_input_device_index = audio.get_default_input_device_info()["index"]
    return default_input_device_index

def recognize_speech_from_mic(recognizer, microphone):
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Listening...")
        audio = recognizer.listen(source)

    try:
        return recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        print("Unknown command: could not understand audio")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")

def laser_callback(scan_data):
    global collision_detected
    # Check for obstacles in front of the robot within a certain threshold distance
    threshold_distance = 0.2  # Adjust this threshold as needed
    min_distance = min(scan_data.ranges)

    if min_distance < threshold_distance:
        collision_detected = True
    else:
        collision_detected = False

def publish_collision_event():
    global collision_detected
    pub = rospy.Publisher('collision_feedback', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # If collision detected, publish feedback
        if collision_detected:
            rospy.loginfo("Collision detected!")
            pub.publish("Collision detected!")
            collision_detected = False  # Reset collision flag

        rate.sleep()

def main():
    rospy.init_node('voice_recognition', anonymous=True)
    recognizer = sr.Recognizer()
    default_input_device_index = get_default_input_device_index()
    print("Default input device index:", default_input_device_index)
    microphone = sr.Microphone(device_index=default_input_device_index)

    # Subscribe to the laser scan topic
    rospy.Subscriber('/scan', LaserScan, laser_callback)

    # Initialize the Twist message and cmd_vel publisher
    twist_msg = Twist()
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Start collision detection thread
    collision_thread = Thread(target=publish_collision_event)
    collision_thread.start()

    while not rospy.is_shutdown():
        response = recognize_speech_from_mic(recognizer, microphone)
        if response:
            print("You said:", response.lower())
            # Example logic to set twist_msg based on recognized command
            if "move forward" in response.lower():
                twist_msg.linear.x = 0.2  # Adjust speed as needed
            elif "move backward" in response.lower():
                twist_msg.linear.x = -0.2
            elif "turn left" in response.lower():
                twist_msg.angular.z = 0.5
            elif "turn right" in response.lower():
                twist_msg.angular.z = -0.5
            elif "stop" in response.lower():
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0

            # Publish the twist message
            cmd_vel_pub.publish(twist_msg)

    collision_thread.join()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass


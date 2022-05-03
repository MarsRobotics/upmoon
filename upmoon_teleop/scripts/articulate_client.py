import rospy
from upmoon_action_msg.msg import ArticulateAction, ArticulateGoal, ArticulateFeedback
from actionlib import SimpleActionClient


NAMESPACE = "action_articulate"
UNPACKED_ANGLE = 3.14
TURN_ANGLE = 2.36


def articulate(angle):
    client = SimpleActionClient(NAMESPACE, ArticulateAction)

    client.wait_for_server()

    goal = ArticulateGoal(lf=angle,
                          lm=angle,
                          lb=angle,
                          rf=angle,
                          rm=angle,
                          rb=angle)

    client.send_goal(goal, feedback_cb=feedback_cb)
    rospy.loginfo("Sent goal to server")

    client.wait_for_result()

    return client.get_result()


def turn():
    client = SimpleActionClient(NAMESPACE, ArticulateAction)

    client.wait_for_server()

    goal = ArticulateGoal(lf=TURN_ANGLE,
                          lm=UNPACKED_ANGLE,
                          lb=TURN_ANGLE,
                          rf=TURN_ANGLE,
                          rm=UNPACKED_ANGLE,
                          rb=TURN_ANGLE)

    client.send_goal(goal, feedback_cb=feedback_cb)

    rospy.loginfo("Sent goal to server")

    client.wait_for_result()

    return client.get_result()


def feedback_cb(fb: ArticulateFeedback):
    rospy.loginfo_once(fb)


def main():
    print("Type 'exit' to exit the shell gracefully.\n")
    print("User Warning: This tool has no soft stop. Be ready to turn off"
          " the robot power if something goes wrong.\n")

    while not rospy.is_shutdown():
        text = input("Enter an absolute angle (radians) or 'turn': ")

        if text == "turn":
            turn()
            continue


        if text == "exit":
            exit()
        
        try:
            angle = float(text)
        except ValueError:
            print("Error: value is not a float")
            continue

        if angle > 6.28 or angle < -6.28:
            print("Error: Angle is out of bounds")
            continue

        articulate(angle)


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_articulate_client")
        main()
    except rospy.ROSInterruptException:
        pass

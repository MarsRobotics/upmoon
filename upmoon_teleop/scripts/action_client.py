import rospy
from upmoon_action_msg.msg import ArticulateAction, ArticulateGoal, ArticulateFeedback
from actionlib import SimpleActionClient


NAMESPACE = "action_articulate"

ANGLE = 0

def action_client():
    client = SimpleActionClient(NAMESPACE, ArticulateAction)

    client.wait_for_server()

    goal = ArticulateGoal(lf=ANGLE,
                          lm=ANGLE,
                          lb=ANGLE,
                          rf=ANGLE,
                          rm=ANGLE,
                          rb=ANGLE)

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    return client.get_result()


def feedback_cb(fb: ArticulateFeedback):
    rospy.loginfo_once(fb)


if __name__ == "__main__":
    try:
        rospy.init_node("teleop_action_client")
        result = action_client()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        pass

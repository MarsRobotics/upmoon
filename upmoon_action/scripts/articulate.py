import rospy
# See http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages#Including_or_Importing_Messages
# for modifications to CMakeLists.txt and package.xml
import upmoon_action_msg.msg
import tf2_ros
import math
from actionlib import SimpleActionServer
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController

from base_action import BaseAction


NODE_NAME = "action_articulate"
NAMESPACE = NODE_NAME
RATE = 4
RATE_TF2 = 10
ROTATION_VELOCITY = 0.8727 # in rad/sec


SWITCH_SERVICE = "/controller_manager/switch_controller"
DEFAULT_DRIVE_CONTROLLERS = ["diff_drive_controller"]
TEMP_DRIVE_CONTROLLERS = ["wheel_lf_controller", "wheel_lm_controller",
                          "wheel_lb_controller", "wheel_rf_controller",
                          "wheel_rm_controller", "wheel_rb_controller", ]


class ArticulateActionServer(BaseAction):
    _feedback = upmoon_action_msg.msg.ArticulateFeedback()
    _result = upmoon_action_msg.msg.ArticulateResult()


    def __init__(self):
        self._name = NAMESPACE
        self._rate = rospy.Rate(RATE)
        self._rate_tf2 = rospy.Rate(RATE_TF2)
        self._server = SimpleActionServer(self._name,
                                          upmoon_action_msg.msg.ArticulateAction,
                                          execute_cb=self.execute_cb,
                                          auto_start=False)

        self._pub_ankle_lf = rospy.Publisher('/ankle_lf_controller/command', Float64, queue_size=10)
        self._pub_ankle_lm = rospy.Publisher('/ankle_lm_controller/command', Float64, queue_size=10)
        self._pub_ankle_lb = rospy.Publisher('/ankle_lb_controller/command', Float64, queue_size=10)
        self._pub_ankle_rf = rospy.Publisher('/ankle_rf_controller/command', Float64, queue_size=10)
        self._pub_ankle_rm = rospy.Publisher('/ankle_rm_controller/command', Float64, queue_size=10)
        self._pub_ankle_rb = rospy.Publisher('/ankle_rb_controller/command', Float64, queue_size=10)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._server.start()


    def execute_cb(self, goal: upmoon_action_msg.msg.ArticulateGoal):
        with self.lock:
            # Calculate duration of action
            curr_angles = self.get_all_angles() 
            goal_angles = [goal.lf, goal.lm, goal.lb, goal.rf, goal.rm, goal.rb]
            net_angle = [g - c for g, c in zip(goal_angles, curr_angles)]

            max_angle = max(net_angle)

            duration = abs(max_angle) / ROTATION_VELOCITY
            self._result.elapsed_time = round(duration)

            # Switch the controller manager for individually controller the drive motors.
            rospy.wait_for_service(SWITCH_SERVICE)
            try:
                switch_controller = rospy.ServiceProxy(SWITCH_SERVICE, SwitchController)
                output = switch_controller(DEFAULT_DRIVE_CONTROLLERS, TEMP_DRIVE_CONTROLLERS, 2)
                rospy.logdebug("Switch controller output: ", output)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                self._server.set_aborted()
                return



            # Execute action.
            self._pub_ankle_lf.publish(Float64(goal.lf))
            self._pub_ankle_lm.publish(Float64(goal.lm))
            self._pub_ankle_lb.publish(Float64(goal.lb))
            self._pub_ankle_rf.publish(Float64(goal.rf))
            self._pub_ankle_rm.publish(Float64(goal.rm))
            self._pub_ankle_rb.publish(Float64(goal.rb))

            # TODO: We need to have a thread per motor because each wheel may articulate a different amount.
            # TODO: Or we can enforce all angles must be 0 or a number that is shared between others.

                        # TODO Spin the motors

            success = True
            while duration > 0:
                if self._server.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._name)
                    self._server.set_preempted()

                    # Action is preempted so update the angle commands to be the current angle.
                    # Note: They may move back a bit because of a delay.
                    curr_angles = self.get_all_angles()
                    self._pub_ankle_lf.publish(Float64(curr_angles[0]))
                    self._pub_ankle_lm.publish(Float64(curr_angles[1]))
                    self._pub_ankle_lb.publish(Float64(curr_angles[2]))
                    self._pub_ankle_rf.publish(Float64(curr_angles[3]))
                    self._pub_ankle_rm.publish(Float64(curr_angles[4]))
                    self._pub_ankle_rb.publish(Float64(curr_angles[5]))

                    success = False
                    break
                    
                # Report feedback.
                self._feedback.remaining_time = round(duration)
                self._server.publish_feedback(self._feedback)

                self._rate.sleep()
                duration -= self._rate.sleep_dur.to_sec()

            # TODO Stop the drive motors.

            # Switch the controller manager back to the default.
            # TODO Maybe don't copy paste this from the upper section.
            rospy.wait_for_service(SWITCH_SERVICE)
            try:
                switch_controller = rospy.ServiceProxy(SWITCH_SERVICE, SwitchController)
                output = switch_controller(TEMP_DRIVE_CONTROLLERS, DEFAULT_DRIVE_CONTROLLERS, 2)
                rospy.logdebug("Switch controller output: ", output)
            except rospy.ServiceException as e:
                rospy.logerr(e)
                self._server.set_aborted()
                return

            # Report success if not preempted.
            if success:
                self._server.set_succeeded(self._result)


    def get_all_angles(self):    
        return [self.get_angle(x) for x in ["lf", "lm", "lb", "rf", "rm" , "rb"]]

    
    def get_angle(self, ankle_name):
        link = "ankle_" + ankle_name + "_link"
        while not rospy.is_shutdown():
            try:
                trans = self._tf_buffer.lookup_transform("base_link", link, rospy.Time())
                break
            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self._rate_tf2.sleep()
                continue
            except tf2_ros.LookupException:
                rospy.logwarn("Could not find TF2 link: ", link)
                continue

        radians = 2 * math.asin(trans.transform.rotation.z)
        return radians


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        server = ArticulateActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

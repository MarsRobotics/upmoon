import rospy
# See http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages#Including_or_Importing_Messages
# for modifications to CMakeLists.txt and package.xml
import upmoon_action_msg.msg
import tf2_ros
import math
import threading
from typing import List
from actionlib import SimpleActionServer
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from upmoon_action.cfg import ArticulateConfig
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController
from urdf_parser_py.urdf import URDF
from tf.transformations import euler_from_quaternion


NODE_NAME = "action_articulate"
NAMESPACE = NODE_NAME
REFRESH_RATE = 4
REFRESH_RATE_TF2 = 10
URDF_TIMEOUT_SEC = 5

LEG_NAMES = ["lf", "lb", "rf", "rb"]

SWITCH_SERVICE = "/controller_manager/switch_controller"
DIFF_DRIVE_CONTROLLERS = ["diff_drive_controller"]
SWERVE_DRIVE_CONTROLLERS = ["wheel_lf_controller", "wheel_lm_controller",
                            "wheel_lb_controller", "wheel_rf_controller",
                            "wheel_rm_controller", "wheel_rb_controller"]

class RobotLeg:

    def __init__(self, leg_name: str, urdf: URDF):
        self.leg_name = leg_name

        ankle_joint_name = 'ankle_' + leg_name + '_joint'

        # Find the joint name in the list of joints.
        try:
            urdf_ankle = next(joint for joint in urdf.joints if joint.name == ankle_joint_name)
        except StopIteration:
            raise RuntimeError("Could not find " + ankle_joint_name + " in urdf description")

        is_left_leg = -1 if leg_name[0] == 'l' else 1

        self.ankle_z_axis = urdf_ankle.axis[2]  # index 2 is z axis and the value is 1 or -1

        self.ankle_offset = self.ankle_z_axis * urdf_ankle.origin.rpy[2]
 
        self.wheel_dir_correction = is_left_leg * self.ankle_z_axis  # Determine the sign (1 or -1)
        self.ankle_topic = rospy.Publisher('/ankle_' + leg_name + '_controller/command', Float64, queue_size=10)
        self.wheel_topic = rospy.Publisher('/wheel_' + leg_name + '_controller/command', Float64, queue_size=10)


class ArticulateActionServer:

    _feedback = upmoon_action_msg.msg.ArticulateFeedback()
    _result = upmoon_action_msg.msg.ArticulateResult()

    def __init__(self):
        self._name = NAMESPACE
        self._rate = rospy.Rate(REFRESH_RATE)
        self._rate_tf2 = rospy.Rate(REFRESH_RATE_TF2)
        self._server = SimpleActionServer(self._name,
                                          upmoon_action_msg.msg.ArticulateAction,
                                          execute_cb=self.execute_cb,
                                          auto_start=False)

        # Parameters loaded by dyanmic_reconfigure
        self.ankle_velocity = None
        self.wheel_velocity = None

        self.dynamic_reconfig = DynamicReconfigureServer(ArticulateConfig, self.dynamic_reconfig_cb)

        # Wait for robot_description param to load.
        timeout = URDF_TIMEOUT_SEC
        while timeout > 0:
            if rospy.has_param('robot_description'):
                break
            rospy.sleep(1)
            timeout -= 1
        else:
            raise RuntimeError("robot_description is not published to the parameter server")

        urdf = URDF.from_parameter_server()
        self._leg_dict = { key : RobotLeg(key, urdf) for key in LEG_NAMES }

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._thread_exit_event = threading.Event()

        self._server.start()

        rospy.loginfo("Articulate action server is ready.")


    def execute_cb(self, goal: upmoon_action_msg.msg.ArticulateGoal):
        rospy.loginfo("Received new request:\n" + str(goal))

        # Calculate duration of action
        curr_angles = self.get_all_angles() 
        goal_angles = [goal.lf - self._leg_dict["lf"].ankle_offset,
                       goal.lb - self._leg_dict["lb"].ankle_offset,
                       goal.rf - self._leg_dict["rf"].ankle_offset,
                       goal.rb - self._leg_dict["rb"].ankle_offset]

        # Do not allow goal angles outside 180 degrees
        goal_angles = [-1.57 if angle < -1.57 else angle for angle in goal_angles] 
        goal_angles = [1.57 if angle > 1.57 else angle for angle in goal_angles]

        net_angles = [g - c for g, c in zip(goal_angles, curr_angles)]

        rospy.loginfo("Curr: " + ",".join(str(x) for x in curr_angles))
        rospy.loginfo("Goal: " + ",".join(str(x) for x in goal_angles))
        rospy.loginfo("Net: " + ",".join(str(x) for x in net_angles))

        threads: List[threading.Thread] = []

        # Switch the controller manager for individually controller the drive motors.
        if not self.enable_swerve_drive():
            return

        # Determine the total time for the goal action.
        max_angle = max(net_angles)
        duration = abs(max_angle) / self.ankle_velocity
        self._result.elapsed_time = round(duration)

        # Start rotating each ankle.
        for leg, curr_angle, goal_angle in zip(self._leg_dict.values(), curr_angles, goal_angles):
            t = threading.Thread(target=self.rotate_ankle, args=(leg, curr_angle, goal_angle))
            threads.append(t)

        for t in threads:
            t.start()

        # Wait until the the action is done.
        success = True
        while duration > 0:
            if self._server.is_preempt_requested():
                rospy.loginfo("Preempted action server.")
                self._server.set_preempted()

                # Stop the threads.
                self._thread_exit_event.set()

                # Action is preempted so update the angle commands to be the current angle.
                # Note: They may move back a bit because of a delay.
                curr_angles = self.get_all_angles()

                for leg, curr_angle in zip(self._leg_dict.values(), curr_angles):
                    leg.ankle_topic.publish(curr_angle)

                success = False
                break
                
            # Report feedback.
            self._feedback.remaining_time = round(duration)
            self._server.publish_feedback(self._feedback)

            self._rate.sleep()
            duration -= self._rate.sleep_dur.to_sec()

        # Clean up the threads.
        for t in threads:
            t.join()

        self._thread_exit_event.clear()

        # Switch the controller manager back to the default.
        if not self.enable_diff_drive():
            return

        # Report success if not preempted.
        if success:
            self._server.set_succeeded(self._result)

    
    def rotate_ankle(self, leg: RobotLeg, curr_angle: float, goal_angle: float):
        rate = rospy.Rate(REFRESH_RATE)

        # Calculate the time to move the ankle joint.    
        net_angle = goal_angle - curr_angle
        duration = abs(net_angle) / self.ankle_velocity

        # Do nothing if we are already at the angle.
        if abs(net_angle) <= 0:
            return

        # Correct wheel velocity by ankle axis of rotation, wheel orientation,
        # and direction of goal.
        # (Back two ankles are opposite in direction of the front and middle, and
        #  left wheels spin in opposite in direction than the right.)
        local_wheel_velocity = self.wheel_velocity * leg.wheel_dir_correction * math.copysign(1, net_angle)

        # Start the ankle motor.
        leg.ankle_topic.publish(goal_angle)

        # Start the drive motor to prevent slipping.
        leg.wheel_topic.publish(local_wheel_velocity)

        # Wait until the ankle has reached the goal position.
        while duration > 0 and not self._thread_exit_event.is_set():
            if duration < rate.sleep_dur.to_sec():
                rospy.sleep(duration)
                break
            else:
                rate.sleep()
                duration -= rate.sleep_dur.to_sec()

        # Stop the drive motor.
        leg.wheel_topic.publish(0)


    def get_all_angles(self):    
        return [self.get_angle(x) for x in LEG_NAMES]

    
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

        q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Old method
        # radians = 2 * math.asin(trans.transform.rotation.z)
        # radians = radians * self._leg_dict[ankle_name].ankle_z_axis

        # Correct ankle by axis rotation.
        yaw = yaw * self._leg_dict[ankle_name].ankle_z_axis

        yaw -= self._leg_dict[ankle_name].ankle_offset

        return yaw


    def enable_diff_drive(self) -> bool:
        return self._switch_controller(DIFF_DRIVE_CONTROLLERS, SWERVE_DRIVE_CONTROLLERS)


    def enable_swerve_drive(self) -> bool:
        return self._switch_controller(SWERVE_DRIVE_CONTROLLERS, DIFF_DRIVE_CONTROLLERS)


    def _switch_controller(self, start_controllers, stop_controllers) -> bool:
        rospy.wait_for_service(SWITCH_SERVICE)
        try:
            switch_controller = rospy.ServiceProxy(SWITCH_SERVICE, SwitchController)
            output = switch_controller(start_controllers, stop_controllers, 2, True, 10)
            rospy.logdebug("Switch controller output: ", output)
        except rospy.ServiceException as e:
            rospy.logerr(e)
            self._server.set_aborted()
            return False
        return True


    def exit(self):
        self._thread_exit_event.set()


    def dynamic_reconfig_cb(self, config, level):
        """Update class variables with dynamic reconfiguration values."""
        self.ankle_velocity = config['ankle_velocity']
        self.wheel_velocity = config['wheel_velocity']
        return config


if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        server = ArticulateActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        server.exit()

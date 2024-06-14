import actionlib
import actionlib_msgs.msg
import control_msgs.msg
from kxr_controller.msg import ServoOnOffAction
from kxr_controller.msg import ServoOnOffGoal
from kxr_controller.msg import Stretch
from kxr_controller.msg import StretchAction
from kxr_controller.msg import StretchGoal
import rospy
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase


class KXRROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, *args, **kwargs):
        namespace = kwargs.get('namespace', '')
        namespace = '/{}'.format(namespace) if namespace else ''
        joint_param = namespace + '/kxr_fullbody_controller/joints'
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.joint_names = rospy.get_param(joint_param, None)
            if self.joint_names is not None:
                break
            rospy.logwarn('Waiting {} set'.format(joint_param))
            rate.sleep()
        super(KXRROSRobotInterface, self).__init__(*args, **kwargs)
        self.servo_on_off_client = actionlib.SimpleActionClient(
            namespace + '/kxr_fullbody_controller/servo_on_off',
            ServoOnOffAction)
        self.servo_on_off_client.wait_for_server()
        self.stretch_client = actionlib.SimpleActionClient(
            namespace + '/kxr_fullbody_controller/stretch_real_interface',
            StretchAction)
        self.stretch_client.wait_for_server()
        self.stretch_topic_name = namespace \
            + '/kxr_fullbody_controller/stretch'

    def servo_on(self, joint_names=None):
        if joint_names is None:
            joint_names = self.joint_names
        goal = ServoOnOffGoal()
        client = self.servo_on_off_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        self.angle_vector(self.angle_vector(), 0.1)
        self.wait_interpolation()
        rospy.sleep(1.0)
        goal.joint_names = joint_names
        goal.servo_on_states = [True] * len(joint_names)
        client.send_goal(goal)

    def servo_off(self, joint_names=None):
        if joint_names is None:
            joint_names = self.joint_names
        goal = ServoOnOffGoal()
        client = self.servo_on_off_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        goal.servo_on_states = [False] * len(joint_names)
        client.send_goal(goal)

    def send_stretch(self, value=127, joint_names=None):
        if joint_names is None:
            joint_names = self.joint_names
        goal = StretchGoal()
        client = self.stretch_client
        if client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            client.cancel_goal()
            client.wait_for_result(timeout=rospy.Duration(10))
        goal.joint_names = joint_names
        goal.stretch = value
        client.send_goal(goal)

    def read_stretch(self):
        return rospy.wait_for_message(
            self.stretch_topic_name, Stretch)

    @property
    def fullbody_controller(self):
        cont_name = 'kxr_fullbody_controller'
        return dict(
            controller_type=cont_name,
            controller_action=cont_name + '/follow_joint_trajectory',
            controller_state=cont_name + '/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=self.joint_names,
        )

    def default_controller(self):
        return [self.fullbody_controller]

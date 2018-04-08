import actionlib
import rospy
import threading

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import RoombaStateStampedArray

"""
RoombaRequest: Wrapper for requests to Roomba Request Executer

Args:
    frame_id: ID of roomba action is to be taken against
    tracking_mode: are we blocking, hitting, or bumping the roomba
    time_to_hold_once_done: time to hold after taking roomba actions
"""
class RoombaRequest(object):
    BLOCK = 1
    HIT = 2
    BUMP = 3

    def __init__(self, frame_id, tracking_mode, time_to_hold_once_done):
        if (tracking_mode != RoombaRequest.BLOCK
                and tracking_mode != RoombaRequest.HIT
                and tracking_mode != RoombaRequest.BUMP):
            raise ValueError, \
                'tracking_mode must be BLOCK, HIT, or BUMP'

        if frame_id is None:
            raise ValueError, \
                'frame_id cannot be None'

        if time_to_hold_once_done < 0.0:
            raise ValueError, \
                'time_to_hold_once_done cannot be negative'

        self.frame_id = frame_id
        self.tracking_mode = tracking_mode
        self.time_to_hold = time_to_hold_once_done

"""
RRE State: Class used by RRE to keep track of its internal state and 
to provide feedback to the user of the RRE
"""
class RoombaRequestExecuterState(object):
    FOLLOWING = 1
    GOING_TO = 2
    BLOCK = 3
    HIT = 4
    RECOVER_FROM_SUCCESS = 5
    HOLD_POSITION = 6
    SUCCESS = 7
    RECOVER_FROM_FAILURE = 8
    FAILED_TASK = 9
    FAILED_RECOVERY = 10
    FAILED_TASK_AND_RECOVERY = 11
    INVALID_STATE = 12

class InvalidRoombaRequestException(Exception):
    pass

"""
Roomba Request Executer: simple state machine that takes care of making
the appropriate task requests to hit, block, or bump a roomba.

Hit: hits the top plate of a roomba
Block: blocks the roomba from its front side
Bump: hovers atop a roomba and blocks it if it turns around

NOTE: If you want to track a roomba, please request a Track Roomba Task
"""

class RoombaRequestExecuter(object):
    _initialized = False
    _lock = threading.Lock()
    _running = False

    def __init__(self):
        raise NotImplementedError, 'Cannot create instances of this class'

    @classmethod
    def init(cls, server_name):
        """
        Roomba Request Executer constructor

        Args:
            roomba_request (RoombaRequest): object with request details from AI
            client (SimpleActionClient): client to send action commands to
            status_callback: callback funtion to provide status updates to
        """
        with cls._lock:
            if cls._initialized:
                return

            # Creates the SimpleActionClient, passing the type of the action
            # (QuadMoveAction) to the constructor. (Look in the action folder)
            cls._client = actionlib.SimpleActionClient(
                server_name,
                QuadMoveAction)

            # roomba_sub = rospy.Subscriber('roomba_states',
            #             RoombaStateStampedArray, cls._roomba_state_callback)

            # Waits until the action server has started up and started
            # listening for goals.
            cls._client.wait_for_server()
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

            cls._initialized = True
            cls._roomba_msg = None
            cls._roomba_turned = False

    @classmethod
    def has_running_task(cls):
        return cls._running

    @classmethod
    def run(cls, roomba_request, status_callback=lambda _: None):
        with cls._lock:
            if not cls._initialized:
                raise InvalidRoombaRequestException, 'Called run before init'
            if cls._running:
                raise InvalidRoombaRequestException, 'Request already running'

            cls._running = True

            next_thread = threading.Thread(
                target=cls._run,
                args=(roomba_request, status_callback))
            next_thread.start()

    @classmethod
    def _roomba_state_callback(cls, msg):
        with cls._lock:
            cls._roomba_msg = msg

    @classmethod
    def _run(cls, roomba_request, status_callback):
        """
        Roomba Request Executer run function

        Description:
            Fires off tasks until the roomba request is finished, canceled, or
            fails.
        """

        # removes the "/base_link" at end of roomba_id
        roomba_id = roomba_request.frame_id.split('/')[0]

        time_to_hold = roomba_request.time_to_hold
        tracking_mode = roomba_request.tracking_mode

        holding = (time_to_hold != 0)

        state = RoombaRequestExecuterState.GOING_TO

        track_sent = False

        status_callback(state)

        while not rospy.is_shutdown():
            """ determining goals """
            if (state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE
                    or state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS):
                goal = QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.logwarn("Recover Height success: {}".format(
                    cls._client.get_result()))

            elif state == RoombaRequestExecuterState.GOING_TO:
                goal = QuadMoveGoal(movement_type="go_to_roomba",
                                    frame_id=roomba_id,
                                    ending_radius=.5)
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                rospy.logwarn("Go to Roomba success: {}".format(
                    cls._client.get_result()))

            elif state == RoombaRequestExecuterState.FOLLOWING:
                if not track_sent:
                    goal = QuadMoveGoal(movement_type="track_roomba",
                                        frame_id=roomba_id,
                                        x_overshoot=-.25,
                                        y_overshoot=-.25)
                    # Sends the goal to the action server.
                    cls._client.send_goal(goal)
                    rospy.logwarn("Following Roomba")
                    track_sent = True

                if cls._roomba_turned:
                    cls._client.cancel_goal()

                # rate limiting
                rospy.sleep(.05)

            elif state == RoombaRequestExecuterState.HIT:
                goal = QuadMoveGoal(movement_type="hit_roomba",
                                    frame_id=roomba_id)
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.logwarn("Hit Roomba success: {}".format(
                    cls._client.get_result()))

            elif state == RoombaRequestExecuterState.BLOCK:
                goal = QuadMoveGoal(movement_type="block_roomba",
                                    frame_id=roomba_id)
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.logwarn("BlockRoomba success: {}".format(
                    cls._client.get_result()))

            elif state == RoombaRequestExecuterState.HOLD_POSITION:
                goal = QuadMoveGoal(movement_type="hold_position",
                                    hold_current_position=True)
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # waits to cancel hold task
                rospy.sleep(time_to_hold)
                cls._client.cancel_goal()
                rospy.logwarn("Hold Position Task canceled")

            """state transitioning"""

            # see http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
            # for status IDs
            if cls._client.get_state() not in [4,5,9]:
                # the task was successful

                # Hit and Block roomba require recovery afterwards
                if state == RoombaRequestExecuterState.HIT:
                    state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                elif state == RoombaRequestExecuterState.BLOCK:
                    state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                # transition after going to a roomba
                elif state == RoombaRequestExecuterState.GOING_TO:
                    if tracking_mode == RoombaRequest.BLOCK:
                        state = RoombaRequestExecuterState.BLOCK
                    elif tracking_mode == RoombaRequest.HIT:
                        state = RoombaRequestExecuterState.HIT
                    else:
                        state = RoombaRequestExecuterState.FOLLOWING

                # bump roomba logic
                elif state == RoombaRequestExecuterState.FOLLOWING:
                    if cls._roomba_turned:
                        state = RoombaRequestExecuterState.BLOCK

                # successfully recovered, so do we hold now
                elif state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                    if holding:
                        state = RoombaRequestExecuterState.HOLD_POSITION
                    else:
                        state = RoombaRequestExecuterState.SUCCESS

                # roomba task failed, but recover succeeded
                elif state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE:
                    state = RoombaRequestExecuterState.FAILED_TASK

                # success after holding
                elif state == RoombaRequestExecuterState.HOLD_POSITION:
                    state = RoombaRequestExecuterState.SUCCESS

                else:
                    rospy.logerr("Roomba Request Executer is in an invalid state")
                    state = RoombaRequestExecuterState.INVALID_STATE
            else:
                if state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE:
                    state = RoombaRequestExecuterState.FAILED_TASK_AND_RECOVERY

                elif state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                    state = RoombaRequestExecuterState.FAILED_RECOVERY

                else:
                    state = RoombaRequestExecuterState.RECOVER_FROM_FAILURE

            status_callback(state)

            if state in (RoombaRequestExecuterState.SUCCESS,
                         RoombaRequestExecuterState.INVALID_STATE,
                         RoombaRequestExecuterState.FAILED_RECOVERY,
                         RoombaRequestExecuterState.FAILED_TASK,
                         RoombaRequestExecuterState.FAILED_TASK_AND_RECOVERY):
                break
        cls._running = False

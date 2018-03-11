import actionlib
import rospy
import threading
import math

from iarc7_motion.msg import QuadMoveGoal, QuadMoveAction
from iarc7_msgs.msg import OdometryArray 

class RoombaRequest(object):
    BLOCK = 1
    HIT = 2
    BUMP = 3

    def __init__(self, frame_id, tracking_mode, time_to_hold_once_done):
        if (tracking_mode != RoombaRequest.BLOCK
                and tracking_mode != RoombaRequest.HIT
                and tracking_mode != RoombaRequest.BUMP):
            raise (ValueError, 
                'tracking_mode must be RoombaRequest.BLOCK or RoombaRequest.HIT or RoombaRequest.BUMP')
        self.frame_id = frame_id
        self.tracking_mode = tracking_mode
        self.time_to_hold = time_to_hold_once_done
        

class RoombaRequestExecuterState(object):
    TRACKING = 1
    BLOCK = 2
    HIT = 3
    BUMP = 4
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

            # Waits until the action server has started up and started
            # listening for goals.
            cls._client.wait_for_server()
            if rospy.is_shutdown():
                raise rospy.ROSInterruptException()

            cls._roombas = None
            cls._roomba_id = None
            cls._last_roomba_vel = None
            cls._rommba_rotating = False

            cls._roomba_status_sub = rospy.Subscriber(
            'roombas', OdometryArray, 
            cls._roomba_callback)

            cls._initialized = True

    @classmethod
    def has_running_task(cls):
        return cls._running

    @classmethod
    def _roomba_callback(cls, data): 
        cls._roombas = data

        if cls._roomba_id is not None: 
            if cls._last_roomba_vel is None:
                for odometry in data.data:
                    if odometry.child_frame_id == cls._roomba_id:
                        roomba_x_velocity = odometry.twist.twist.linear.x
                        roomba_y_velocity = odometry.twist.twist.linear.y
                        cls._last_roomba_vel = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)
            else: 
                for odometry in data.data:
                    if odometry.child_frame_id == cls._roomba_id:
                        roomba_x_velocity = odometry.twist.twist.linear.x
                        roomba_y_velocity = odometry.twist.twist.linear.y
                        current_roomba_vel = math.sqrt(roomba_x_velocity**2 + roomba_y_velocity**2)

                cls._rommba_rotating = (abs(current_roomba_vel - cls._last_roomba_vel)>.15
                    or current_roomba_vel <= .15)

    @classmethod
    def run(cls, roomba_request, status_callback=lambda _: None):
        with cls._lock:
            if not cls._initialized:
                raise InvalidRoombaRequestException, 'Called run before init'
            if cls._running:
                raise InvalidRoombaRequestException, 'Request already running'

            cls._last_roomba_vel = None
            cls._rommba_rotating = False
            cls._roomba_id = roomba_request.frame_id
            cls._running = True

            next_thread = threading.Thread(
                target=cls._run,
                args=(roomba_request, status_callback))
            next_thread.start()

    @classmethod
    def _run(cls, roomba_request, status_callback):
        """
        Roomba Request Executer run fucntion

        Description:
            Fires off tasks until the roomba request is finished, canceled, or
            fails.
        """

        # removes the "/frame_id" at end of roomba_id
        roomba_id = roomba_request.frame_id.split('/')[0]

        time_to_hold = roomba_request.time_to_hold

        tracking_mode = roomba_request.tracking_mode

        holding = (time_to_hold != 0)

        if tracking_mode == RoombaRequest.HIT:
            state = RoombaRequestExecuterState.HIT
        elif tracking_mode == RoombaRequest.BUMP:
            state = RoombaRequestExecuterState.BUMP
        else: 
            state = RoombaRequestExecuterState.BLOCK

        status_callback(state)

        while not rospy.is_shutdown():

            tracking = False

            # determining goals
            if (state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE
                    or state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS):
                # this is where the height recover task will be called
                goal = QuadMoveGoal(movement_type="height_recovery")
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.logwarn("Recover Height success: {}".format(
                    cls._client.get_result()))

            elif state == RoombaRequestExecuterState.HIT:
                goal = QuadMoveGoal(movement_type="hit_roomba",
                                    frame_id=roomba_id)
                # Sends the goal to the action server.
                cls._client.send_goal(goal)
                # Waits for the server to finish performing the action.
                cls._client.wait_for_result()
                if rospy.is_shutdown():
                    raise rospy.ROSInterruptException()
                rospy.logwarn("HITRoomba success: {}".format(
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

            elif state == RoombaRequestExecuterState.BUMP:
                if not tracking: 
                    goal = QuadMoveGoal(movement_type="track_roomba", frame_id=roomba_id, 
                                        x_overshoot=-.25, y_overshoot=-.25)
                    # Sends the goal to the action server.
                    cls._client.send_goal(goal)
                    tracking = True

                if cls._rommba_rotating: 
                    cls._client.cancel_goal()
                    state = RoombaRequestExecuterState.BLOCK


            # state transitioning
            if (state != RoombaRequestExecuterState.HOLD_POSITION
                    and (not cls._client.get_result()
                        or not cls._client.get_result().success)):
                if state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE:
                    state = RoombaRequestExecuterState.FAILED_TASK_AND_RECOVERY
                elif state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                    state = RoombaRequestExecuterState.FAILED_RECOVERY
                else:
                    state = RoombaRequestExecuterState.RECOVER_FROM_FAILURE
            else:
                if state == RoombaRequestExecuterState.HIT:
                    state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                elif state == RoombaRequestExecuterState.BLOCK:
                    state = RoombaRequestExecuterState.RECOVER_FROM_SUCCESS

                elif state == RoombaRequestExecuterState.BUMP: 
                    pass

                elif state == RoombaRequestExecuterState.RECOVER_FROM_SUCCESS:
                    if holding:
                        state = RoombaRequestExecuterState.HOLD_POSITION
                    else:
                        state = RoombaRequestExecuterState.SUCCESS

                elif state == RoombaRequestExecuterState.RECOVER_FROM_FAILURE:
                    state = RoombaRequestExecuterState.FAILED_TASK

                elif state == RoombaRequestExecuterState.HOLD_POSITION:
                    state = RoombaRequestExecuterState.SUCCESS

                else:
                    rospy.logerr("Roomba Controller is in an invalid state")
                    state = RoombaRequestExecuterState.INVALID_STATE

            status_callback(state)

            if state in (RoombaRequestExecuterState.SUCCESS,
                         RoombaRequestExecuterState.INVALID_STATE,
                         RoombaRequestExecuterState.FAILED_RECOVERY,
                         RoombaRequestExecuterState.FAILED_TASK,
                         RoombaRequestExecuterState.FAILED_TASK_AND_RECOVERY):
                break
        cls._running = False
        cls._roomba_id = None

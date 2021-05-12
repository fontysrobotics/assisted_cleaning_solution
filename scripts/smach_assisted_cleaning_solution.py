#!/usr/bin/env python

import roslib; roslib.load_manifest('assisted_cleaning_solution')

import rospy
from smach import StateMachine, Concurrence, State, Sequence, CBState
from smach_ros import ServiceState, SimpleActionState, MonitorState, IntrospectionServer, ActionServerWrapper

from assisted_cleaning_solution.msg import Task, PositioningPlinthsAction, CleanPlinthsAction, FindChairsAction, ReactionObstacleAction, MoveAroundObstacleAction, GoToChairAction, MoveAroundChairAction, GoToPathAction
from assisted_cleaning_solution.srv import SelectCoords, GroupChairs, CheckChairs

from move_base_msgs.msg import MoveBaseAction

from util import receive_task, cancel_goal, path_clear
from callback import *

def task():
    rospy.init_node('smach_assisted_cleaning_solution')

    sm_top = StateMachine(outcomes = [])
    with sm_top:
    # ---------------------------------------------------------------------
    # Reset so everything can be interuppted
        StateMachine.add('RESET', 
            MonitorState('/task', 
                Task, 
                cond_cb = lambda userdata,msg: not monitor(userdata,msg)), 
            transitions = {'valid':'RESET', 
                           'invalid':'EVERYTHING',
                           'preempted':'RESET'})

    # ---------------------------------------------------------------------
    # Concurrence container to monitor the task msgs from GUI and start the actions
        sm_con = Concurrence(outcomes = ['succeeded'],
            default_outcome = 'succeeded',
            child_termination_cb = lambda state_outcomes: True,
            outcome_map = {'succeeded':{'MONITOR_MSG':'succeeded',
                                        'ACTION':'preempted'}})

        StateMachine.add('EVERYTHING', 
            sm_con, 
            transitions = {'succeeded':'RESET'})

        with sm_con:
    # ---------------------------------------------------------------------
    # Container for Monitoring if new message received
            sm_preempt = StateMachine(outcomes = ['succeeded'])
            Concurrence.add('MONITOR_MSG', sm_preempt)

            with sm_preempt:
                StateMachine.add('MSG_CHECK',
                    MonitorState('/task', Task, cond_cb = monitor),
                    transitions = {'invalid':'CANCEL_GOAL',
                                   'valid':'MSG_CHECK',
                                   'preempted':'MSG_CHECK'})
                
                StateMachine.add('CANCEL_GOAL', cancel_goal(),
                    transitions = {'cancel_outcome':'succeeded'})


    # ---------------------------------------------------------------------
    # Container for action and the values that pass between states
            sm_action = StateMachine(outcomes = ['aborted', 'preempted'])
            Concurrence.add('ACTION', sm_action)

            sm_action.userdata.coords = [0, 0, 0, 1]
            sm_action.userdata.new_task = 0
            sm_action.userdata.previous_task = 0
            sm_action.userdata.room_number = 0

    # ---------------------------------------------------------------------
    # Choosing action
            with sm_action:
                StateMachine.add('RECEIVE_TASK',
                    receive_task(), 
                    transitions = {'nextRoom':'NEXT_ROOM', 
                                   'plinths':'PLINTHS', 
                                   'chairs':'CHAIRS', 
                                   'not_received':'RECEIVE_TASK',
                                   'preempted':'preempted'},
                    remapping = {'new_task':'new_task',
                                 'previous_task':'previous_task'})

                StateMachine.add('UPDATE_TASK',
                    CBState(update_task, 
                        output_keys = ['previous_task']),
                    transitions = {'preempted':'preempted'},
                    remapping = {'previous_task':'previous_task'})

                StateMachine.add('TASK_COMPLETED',
                    CBState(task_completed, 
                        output_keys = ['previous_task']),
                    transitions = {'succeeded':'RECEIVE_TASK'},
                    remapping = {'previous_task':'previous_task'})

    # ---------------------------------------------------------------------
    # Container for navigation to coordinates in room
                sm_room = StateMachine(outcomes = ['aborted', 'preempted', 'succeeded'], 
                    input_keys = ['coords', 'new_task', 'room_number'], 
                    output_keys = ['coords', 'room_number'])
                StateMachine.add('NEXT_ROOM', 
                    sm_room,
                    transitions = {'succeeded':'TASK_COMPLETED',
                                   'preempted':'UPDATE_TASK'})
                
                with sm_room:
                    StateMachine.add('SELECT_ROOM', 
                        ServiceState('select_coords_service', 
                            SelectCoords,
                            request_cb = select_room,
                            input_keys = ['room', 'coords'],
                            response_slots = ['coords', 'count']),
                        transitions = {'succeeded':'GO_TO_ROOM',
                                       'preempted':'preempted'},
                        remapping = {'room':'new_task',
                                     'coords':'coords',
                                     'count':'room_number'})
                    
                    StateMachine.add('GO_TO_ROOM',
                        SimpleActionState('move_base',
                            MoveBaseAction,
                            goal_cb = goal_planner,
                            result_cb = result_planner,
                            input_keys = ['array']),
                        transitions = {'succeeded':'succeeded',
                                       'preempted':'preempted'},
                        remapping = {'array':'coords'})

    # ---------------------------------------------------------------------
    # Container for cleaning plinths
                sm_plinths = StateMachine(outcomes = ['aborted', 'succeeded', 'preempted'], 
                    input_keys = ['coords'])
                StateMachine.add('PLINTHS', 
                    sm_plinths, 
                    transitions = {'succeeded':'TASK_COMPLETED',
                                   'preempted':'UPDATE_TASK'})

                with sm_plinths:
                    StateMachine.add('POSITIONING_FOR_PLINTHS',
                        SimpleActionState('positioning_for_plinths_action',
                            PositioningPlinthsAction),
                        transitions = {'succeeded':'FOLLOW_PLINTHS',
                                       'preempted':'preempted'})

                    StateMachine.add('FOLLOW_PLINTHS',
                        SimpleActionState('follow_plinths_action',
                            CleanPlinthsAction,
                            result_cb = result_done_or_obstacle,
                            goal_slots = ['coords'],
                            result_slots = ['done']),
                        transitions = {'succeeded':'succeeded',
                                       'preempted':'preempted',
                                       'obstacle':'OBSTACLE'},
                        remapping = {'coords':'coords'})

                    StateMachine.add('OBSTACLE',
                        SimpleActionState('reaction_obstacle_service',
                            ReactionObstacleAction,
                            result_cb = result_obstacle,
                            result_slots = ['reaction']),
                        transitions = {'turn':'MOVE_AROUND',
                                       'continue':'CLEAR_TO_CONTINUE',
                                       'preempted':'preempted'})

                    StateMachine.add('MOVE_AROUND',
                        SimpleActionState('move_around_obstacle_action',
                            MoveAroundObstacleAction),
                        transitions = {'succeeded':'FOLLOW_PLINTHS',
                                       'preempted':'preempted'})

                    StateMachine.add('CLEAR_TO_CONTINUE',
                        path_clear(),
                        transitions = {'clear':'FOLLOW_PLINTHS',
                                       'not_clear':'CLEAR_TO_CONTINUE',
                                       'time_up':'OBSTACLE',
                                       'move_around':'MOVE_AROUND',
                                       'preempted':'preempted'})
                        
    # ---------------------------------------------------------------------
    # Container for cleaning chairs and the values used between states
                sm_chairs = StateMachine(outcomes = ['aborted', 'succeeded', 'preempted'],
                    input_keys = ['room_number'])
                StateMachine.add('CHAIRS', 
                    sm_chairs, 
                    transitions = {'succeeded':'TASK_COMPLETED',
                                   'preempted':'UPDATE_TASK'})

                sm_chairs.userdata.count_steps = 0
                sm_chairs.userdata.next_pose_around_chair = []

                sm_chairs.userdata.x_coords_chair = []
                sm_chairs.userdata.y_coords_chair = []
                sm_chairs.userdata.x_coord_chair = 0
                sm_chairs.userdata.y_coord_chair = 0

                sm_chairs.userdata.times_cleaned = 0

                with sm_chairs:
                    StateMachine.add('POSE_AROUND_CHAIRS',
                        ServiceState('select_coords_service',
                            SelectCoords,
                            request_cb = select_path,
                            response_cb = response_next_state,
                            input_keys = ['count_steps', 
                                          'room_number'],
                            response_slots = ['count', 
                                              'coords']),
                        transitions = {'succeeded':'DRIVE_AROUND_CHAIRS',
                                       'preempted':'preempted',
                                       'done_finding_chair':'CHECK_iF_CHAIRS_NEARBY'},
                        remapping = {'count_steps':'count_steps',
                                     'coords':'next_pose_around_chair',
                                     'count':'count_steps'})

                    StateMachine.add('DRIVE_AROUND_CHAIRS',
                        SimpleActionState('move_base',
                            MoveBaseAction,
                            goal_cb = goal_planner,
                            result_cb = result_planner,
                            input_keys = ['array']),
                        transitions = {'succeeded':'FIND_CHAIRS',
                                       'preempted':'preempted'},
                        remapping = {'array':'next_pose_around_chair'})

                    StateMachine.add('FIND_CHAIRS',
                        SimpleActionState('find_chairs_action',
                            FindChairsAction,
                            result_cb = result_find_chairs,
                            result_slots = ['x_coords_chair',
                                            'y_coords_chair'],
                            input_keys = ['count_steps']),
                        transitions = {'succeeded':'POSE_AROUND_CHAIRS',
                                       'preempted':'preempted',
                                       'done_finding_chair':'GROUP_CHAIRS'},
                        remapping = {'count_steps':'count_steps',
                                     'x_coords_chair':'x_coords_chair',
                                     'y_coords_chair':'y_coords_chair'})

                    StateMachine.add('GROUP_CHAIRS',
                        ServiceState('group_chairs_service', 
                            GroupChairs,
                            request_slots = ['x_coords_chair',
                                             'y_coords_chair'],
                            response_slots = ['x_coords_chair_group',
                                              'y_coords_chair_group']),
                        transitions = {'succeeded':'CHECK_iF_CHAIRS_NEARBY',
                                       'preempted':'preempted'},
                        remapping = {'x_coords_chair':'x_coords_chair',
                                     'y_coords_chair':'y_coords_chair',
                                     'x_coords_chair_group':'x_coords_chair',
                                     'y_coords_chair_group':'y_coords_chair'})
                    
                    StateMachine.add('CHECK_iF_CHAIRS_NEARBY',
                        ServiceState('check_chairs_service',
                            CheckChairs,
                            response_cb = response_check_chairs,
                            request_slots = ['x_coords_chair',
                                             'y_coords_chair'],
                            response_slots = ['next_state',
                                              'x_coord_chair',
                                              'y_coord_chair'],
                            input_keys = ['count_steps'],
                            output_keys = ['count_steps']),
                        transitions = {'succeeded':'succeeded',
                                       'preempted':'preempted',
                                       'none_nearby':'POSE_AROUND_CHAIRS_BACK',
                                       'nearby':'GO_TO_CHAIR'},
                        remapping = {'x_coords_chair':'x_coords_chair',
                                     'y_coords_chair':'y_coords_chair',
                                     'x_coord_chair':'x_coord_chair',
                                     'y_coord_chair':'y_coord_chair',
                                     'count_steps':'count_steps'})

                    StateMachine.add('POSE_AROUND_CHAIRS_BACK',
                        ServiceState('select_coords_service',
                            SelectCoords,
                            request_cb = select_path,
                            response_cb = response_pose_chair,
                            input_keys = ['count_steps', 
                                          'room_number'],
                            output_keys = ['count', 
                                           'coords']),
                        transitions = {'succeeded':'DRIVE_AROUND_CHAIRS_BACK',
                                       'end_of_path':'succeeded',
                                       'preempted':'preempted'},
                        remapping = {'count_steps':'count_steps',
                                     'coords':'next_pose_around_chair',
                                     'count':'count_steps'})

                    StateMachine.add('DRIVE_AROUND_CHAIRS_BACK',
                        SimpleActionState('move_base',
                            MoveBaseAction,
                            goal_cb = goal_planner,
                            result_cb = result_planner,
                            input_keys = ['array']),
                        transitions = {'succeeded':'CHECK_iF_CHAIRS_NEARBY',
                                       'preempted':'preempted'},
                        remapping = {'array':'next_pose_around_chair'})

                    StateMachine.add('GO_TO_CHAIR',
                        SimpleActionState('go_to_chair_action',
                            GoToChairAction,
                            goal_slots = ['x_coord_chair',
                                          'y_coord_chair']),
                        transitions = {'succeeded':'COUNT_CLEAN_CHAIR',
                                       'preempted':'preempted'},
                        remapping = {'x_coord_chair':'x_coord_chair',
                                     'y_coord_chair':'y_coord_chair'})

                    StateMachine.add('COUNT_CLEAN_CHAIR',
                        CBState(count_clean_chair, 
                            input_keys = ['times_cleaned'],
                            output_keys = ['times_cleaned']),
                        transitions = {'succeeded':'MOVE_AROUND_CHAIR',
                                       'done_cleaning':'REMOVE_CHAIR_FROM_LIST'},
                        remapping = {'times_cleaned':'times_cleaned'})

                    # Put state for clean chair here:

                    # The coordinates of the chairs are not accurate enough for this state
                    StateMachine.add('MOVE_AROUND_CHAIR',
                        SimpleActionState('move_around_chair_action',
                            MoveAroundChairAction),
                        transitions = {'succeeded':'COUNT_CLEAN_CHAIR',
                                       'preempted':'preempted'})

                    StateMachine.add('REMOVE_CHAIR_FROM_LIST', 
                        CBState(remove_chair_from_list),
                        transitions = {'succeeded':'GO_TO_PATH'},
                        remapping = {'x_coords_chair':'x_coords_chair',
                                     'y_coords_chair':'y_coords_chair',
                                     'x_coord_chair':'x_coord_chair',
                                     'y_coord_chair':'y_coord_chair'})

                    StateMachine.add('GO_TO_PATH',
                        SimpleActionState('go_to_path_action',
                            GoToPathAction,
                            goal_slots = ['pose_around_chair']),
                        transitions = {'succeeded':'CHECK_iF_CHAIRS_NEARBY',
                                       'preempted':'preempted'},
                        remapping = {'pose_around_chair':'next_pose_around_chair'})

                    
    # ---------------------------------------------------------------------
    # Smach viewer and Execute state machine
    sis = IntrospectionServer('server_name', sm_top, '/SM_START')
    sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    try:
        task()
    except rospy.ROSInterruptException:
        pass
/*
 * Copyright (C) 2011 Alexander Perzylo, Technische Universität München
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.actionlib.client;

import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.state.CommState;
import org.ros.exception.RosException;
import org.ros.internal.message.Message;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;


import java.util.ArrayList;

/**
 * A CommStateMachine monitors the communication between an action client and an
 * action server and stores the state of progress of a goal. CommStateMachines
 * get used by {@link ClientGoalHandle}s.
 * 
 * @author Alexander C. Perzylo, perzylo@cs.tum.edu
 * 
 * @param <T_ACTION_FEEDBACK>
 *          action feedback message
 * @param <T_ACTION_GOAL>
 *          action goal message
 * @param <T_ACTION_RESULT>
 *          action result message
 * @param <T_FEEDBACK>
 *          feedback message
 * @param <T_GOAL>
 *          goal message
 * @param <T_RESULT>
 *          result message
 */
public class CommStateMachine<T_ACTION_FEEDBACK extends Message, T_ACTION_GOAL extends Message, T_ACTION_RESULT extends Message, T_FEEDBACK extends Message, T_GOAL extends Message, T_RESULT extends Message> {
  /**
   * Client this goal is associated with.
   */
  protected ActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> actionClient;

  /**
   * The action client callbacks, which get called when transitions of this
   * CommStateMachine's state occur or feedback messages are processed
   */
  protected ActionClientCallbacks<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> callbacks;

  /**
   * The action goal this CommStateMachine is associated with
   */
  protected T_ACTION_GOAL actionGoal;

  /**
   * The action goal's GoalID
   */
  protected String actionGoalID;

  /**
   * The current state of the CommStateMachine
   */
  protected CommState commState;

  /**
   * The latest GoalStatus received
   */
  protected GoalStatus latestGoalStatus;

  /**
   * The latest action result message received
   */
  protected T_ACTION_RESULT latestResult;

  /**
   * The action specification
   */
  protected ActionSpec<?, T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> spec;

  /**
   * Constructor used to create a CommStateMachine, which gets linked to the
   * given action goal message and action specification. The
   * ActionClientCallbacks parameter defines the callback methods to call, when
   * transitions of the CommStateMachine's state occur or feedback messages are
   * processed. If no callbacks shall be called the parameter may be set to
   * NULL.
   * 
   * @param actionGoal
   *          The action goal message this CommStateMachine shall monitor.
   * @param callbacks
   *          The callbacks to call on state transitions or processed feedback
   *          messages
   * @param spec
   *          The action specification
   */
  public CommStateMachine(
      T_ACTION_GOAL actionGoal,
      ActionClientCallbacks<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> callbacks,
      ActionSpec<?, T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> spec,
      ActionClient<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> actionClient)
      throws RosException {

    this.actionGoal = actionGoal;
    this.actionGoalID = spec.getGoalIDFromActionGoal(actionGoal).getId();
    this.callbacks = callbacks;
    this.latestGoalStatus = null;
    this.spec = spec;
    this.commState = new CommState(CommState.StateEnum.WAITING_FOR_GOAL_ACK);
    this.actionClient = actionClient;
  }

  /**
   * Gets this CommStateMachine's associated action goal message.
   * 
   * @return The action goal message
   */
  public T_ACTION_GOAL getActionGoal() {
    return actionGoal;
  }

  /**
   * Gets the CommStateMachine's current state.
   * 
   * @return The current state
   */
  public synchronized CommState getCommState() {
    return commState;
  }

  /**
   * Gets the latest GoalStatus.
   * 
   * @return GoalStatus message
   */
  public synchronized GoalStatus getGoalStatus() {
    return latestGoalStatus;
  }

  /**
   * Gets the latest action result message.
   * 
   * @return Action result message
   */
  public synchronized T_RESULT getResult() throws RosException {
    if (latestResult != null) {
      return spec.getResultFromActionResult(latestResult);
    }

    return null;
  }

  /**
   * Searches in a list of GoalStatus messages for a specific GoalStatus, whose
   * GoalID matches the GoalID of this CommStateMachine's action goal.
   * 
   * @param listStatus
   *          list of GoalStatus messages
   * @return The GoalStatus message. If no matching GoalStatus message could be
   *         found, NULL is returned.
   */
  protected GoalStatus findGoalStatus(ArrayList<GoalStatus> listStatus) {

    GoalStatus status = null;
    for (GoalStatus gs : listStatus) {
      if (gs.getGoalId().getId().equals(actionGoalID)) {
        status = gs;
        break;
      }
    }
    return status;

  }

  /**
   * Extracts feedback messages from received action feedback messages and calls
   * the action client's callback method for every feedback message processed.
   * If the action feedback message's GoalID does not match the GoalID of this
   * CommStateMachine's action goal, this method does nothing.
   * 
   * @param actionFeedback
   *          The action feedback message
   * @param gh
   *          The GoalHandle associated with the goal on which the action
   *          feedback message is received
   */
  public
      void
      updateFeedback(
          T_ACTION_FEEDBACK actionFeedback,
          ClientGoalHandle<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> gh)
          throws RosException {

    if (actionGoalID.equals(spec.getGoalStatusFromActionFeedback(actionFeedback).getGoalId().getId())
        && callbacks != null) {
      callbacks.feedbackCallback(gh, spec.getFeedbackFromActionFeedback(actionFeedback));

    }

  }

  /**
   * Extracts and stores the GoalStatus and result messages from the action
   * result message. It updates the current state of this CommStateMachine to
   * 'DONE' and calls the
   * {@link #updateStatus(GoalStatusArray, ClientGoalHandle)} method using the
   * extracted GoalStatus message. If the GoalID of the action result message's
   * GoalStatus does not match the GoalID of this CommStateMachine's action
   * goal, this method does nothing.
   * 
   * @param actionResult
   *          The action result message
   * @param gh
   *          The GoalHandle associated with the goal on which the action result
   *          message is received
   */
  public synchronized
      void
      updateResult(
          T_ACTION_RESULT actionResult,
          ClientGoalHandle<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> gh)
          throws RosException {

    if (!actionGoalID.equals(spec.getGoalStatusFromActionResult(actionResult).getGoalId().getId())) {
      return;
    }

    latestGoalStatus = spec.getGoalStatusFromActionResult(actionResult);
    latestResult = actionResult;

    switch (commState.getState()) {
    case WAITING_FOR_GOAL_ACK:
    case PENDING:
    case ACTIVE:
    case WAITING_FOR_RESULT:
    case WAITING_FOR_CANCEL_ACK:
    case RECALLING:
    case PREEMPTING:
      NodeConfiguration nc = NodeConfiguration.newPrivate();
      MessageFactory mf = nc.getTopicMessageFactory();
      GoalStatusArray statusArray = mf.newFromType(GoalStatusArray._TYPE);
      java.util.ArrayList<actionlib_msgs.GoalStatus> sl = new java.util.ArrayList<actionlib_msgs.GoalStatus>(statusArray.getStatusList());
      sl.add(spec.getGoalStatusFromActionResult(actionResult));
      statusArray.setStatusList(sl);
      updateStatus(statusArray, gh);
      transitionToState(CommState.StateEnum.DONE, gh);
      break;
    case DONE:
      actionClient.getNode().getLog()
          .error("[CommStateMachine] Got a result when we were already in the 'DONE' state");
      break;
    default:
      actionClient.getNode().getLog()
          .error("[CommStateMachine] Unknown comm state '" + commState + "'");
      break;
    }

  }

  /**
   * Uses the {@link #findGoalStatus(ArrayList)} method to get the right
   * GoalStatus message from the given list of GoalStatus messages. Based on
   * this GoalStatus the CommStateMachine may transition to a new state using
   * the
   * {@link #transitionToState(ros.actionlib.state.CommState.StateEnum, ClientGoalHandle)}
   * method. If no matching GoalStatus could be found, this method does nothing.
   * 
   * @param gsa
   *          An array of GoalStatus messages
   * @param gh
   *          The GoalHandle associated with the goal on which the GoalStatus
   *          messages are received
   */
  public synchronized
      void
      updateStatus(
          GoalStatusArray gsa,
          ClientGoalHandle<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> gh)
          throws RosException {

    GoalStatus goalStatus = findGoalStatus(new ArrayList<actionlib_msgs.GoalStatus>(gsa.getStatusList()));

    // It's possible to receive old GoalStatus messages over the wire, even
    // after receiving a result with a terminal state. Thus, we want to
    // ignore all statuses that we get after we're done, because they are
    // irrelevant. (See trac #2721)
    if (commState.equals(CommState.StateEnum.DONE)) {
      return;
    }

    if (goalStatus != null) {
      latestGoalStatus = goalStatus;
    } else {
      if (!(commState.equals(CommState.StateEnum.WAITING_FOR_GOAL_ACK)
          || commState.equals(CommState.StateEnum.WAITING_FOR_RESULT) || commState
          .equals(CommState.StateEnum.DONE))) {

        actionClient.getNode().getLog().warn("[CommStateMachine] Transitioning goal to 'LOST'");
        latestGoalStatus.setStatus(GoalStatus.LOST);
        transitionToState(CommState.StateEnum.DONE, gh);
      }
      return;
    }

    switch (commState.getState()) {
    case WAITING_FOR_GOAL_ACK:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        transitionToState(CommState.StateEnum.PENDING, gh);
        break;
      case GoalStatus.ACTIVE:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        break;
      case GoalStatus.PREEMPTED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.SUCCEEDED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.ABORTED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.REJECTED:
        transitionToState(CommState.StateEnum.PENDING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.RECALLED:
        transitionToState(CommState.StateEnum.PENDING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        break;
      case GoalStatus.RECALLING:
        transitionToState(CommState.StateEnum.PENDING, gh);
        transitionToState(CommState.StateEnum.RECALLING, gh);
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case PENDING:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        break;
      case GoalStatus.ACTIVE:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        break;
      case GoalStatus.PREEMPTED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.SUCCEEDED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.ABORTED:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.REJECTED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.RECALLED:
        transitionToState(CommState.StateEnum.RECALLING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        transitionToState(CommState.StateEnum.ACTIVE, gh);
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        break;
      case GoalStatus.RECALLING:
        transitionToState(CommState.StateEnum.RECALLING, gh);
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case ACTIVE:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'ACTIVE' to 'PENDING'");
        break;
      case GoalStatus.ACTIVE:
        break;
      case GoalStatus.REJECTED:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'ACTIVE' to 'REJECTED'");
        break;
      case GoalStatus.RECALLING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'ACTIVE' to 'RECALLING'");
        break;
      case GoalStatus.RECALLED:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'ACTIVE' to 'RECALLED'");
        break;
      case GoalStatus.PREEMPTED:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case WAITING_FOR_RESULT:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'WAITING_FOR_RESULT' to 'PENDING'");
        break;
      case GoalStatus.PREEMPTING:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Invalid transition from 'WAITING_FOR_RESULT' to 'PREEMPTING'");
        break;
      case GoalStatus.RECALLING:
        actionClient
            .getNode()
            .getLog()
            .error("[CommStateMachine] Invalid transition from 'WAITING_FOR_RESULT' to 'RECALLING'");
        break;
      case GoalStatus.ACTIVE:
      case GoalStatus.PREEMPTED:
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
      case GoalStatus.REJECTED:
      case GoalStatus.RECALLED:
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case WAITING_FOR_CANCEL_ACK:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        break;
      case GoalStatus.ACTIVE:
        break;
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
      case GoalStatus.PREEMPTED:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.RECALLED:
        transitionToState(CommState.StateEnum.RECALLING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.REJECTED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        break;
      case GoalStatus.RECALLING:
        transitionToState(CommState.StateEnum.RECALLING, gh);
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case RECALLING:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'RECALLING' to 'PENDING'");
        break;
      case GoalStatus.ACTIVE:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'RECALLING' to 'ACTIVE'");
        break;
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
      case GoalStatus.PREEMPTED:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.RECALLED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.REJECTED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        transitionToState(CommState.StateEnum.PREEMPTING, gh);
        break;
      case GoalStatus.RECALLING:
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case PREEMPTING:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'PREEMPTING' to 'PENDING'");
        break;
      case GoalStatus.ACTIVE:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'PREEMPTING' to 'ACTIVE'");
        break;
      case GoalStatus.REJECTED:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'PREEMPTING' to 'REJECTED'");
        break;
      case GoalStatus.RECALLING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'PREEMPTING' to 'RECALLING'");
        break;
      case GoalStatus.RECALLED:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'PREEMPTING' to 'RECALLED'");
        break;
      case GoalStatus.PREEMPTED:
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
        transitionToState(CommState.StateEnum.WAITING_FOR_RESULT, gh);
        break;
      case GoalStatus.PREEMPTING:
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    case DONE:

      switch (goalStatus.getStatus()) {
      case GoalStatus.PENDING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'DONE' to 'PENDING'");
        break;
      case GoalStatus.ACTIVE:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'DONE' to 'ACTIVE'");
        break;
      case GoalStatus.RECALLING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'DONE' to 'RECALLING'");
        break;
      case GoalStatus.PREEMPTING:
        actionClient.getNode().getLog()
            .error("[CommStateMachine] Invalid transition from 'DONE' to 'PREEMPTING'");
        break;
      case GoalStatus.PREEMPTED:
      case GoalStatus.SUCCEEDED:
      case GoalStatus.ABORTED:
      case GoalStatus.RECALLED:
      case GoalStatus.REJECTED:
        break;
      default:
        actionClient
            .getNode()
            .getLog()
            .error(
                "[CommStateMachine] Got unknown goal status '" + goalStatus.getStatus()
                    + "' from the DefaultActionServer");
        break;
      }
      break;

    default:
      actionClient.getNode().getLog()
          .error("[CommStateMachine] Unknown comm state '" + commState + "'");
      break;
    }

  }

  /**
   * Transitions the CommStateMachine to the given state. If an
   * ActionClientCallbacks object is registered with this CommStateMachine, the
   * corresponding callback method gets called on every transition.
   * 
   * @param state
   *          The new state
   * @param gh
   *          The GoalHandle associated with the goal this CommStateMachine is
   *          monitoring
   */
  public synchronized
      void
      transitionToState(
          CommState.StateEnum state,
          ClientGoalHandle<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> gh)
          throws RosException {

    actionClient.getNode().getLog()
        .debug("[CommStateMachine] Trying to transition to '" + state + "'");
    commState.setState(state);
    if (callbacks != null) {
      callbacks.transitionCallback(gh);
    }

  }

}

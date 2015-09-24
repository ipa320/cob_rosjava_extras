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

package org.ros.actionlib.server;

import org.ros.actionlib.ActionSpec;
import org.ros.actionlib.util.GoalIDGenerator;
import org.ros.exception.RosException;
import org.ros.internal.message.Message;

import org.ros.node.NodeConfiguration;
import org.ros.message.MessageFactory;

import org.ros.message.Time;
import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;

/**
 * Track the status of a goal.
 */
public class StatusTracker<T_ACTION_FEEDBACK extends Message, T_ACTION_GOAL extends Message, T_ACTION_RESULT extends Message, T_FEEDBACK extends Message, T_GOAL extends Message, T_RESULT extends Message> {

  private DefaultActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> server;
  public final T_ACTION_GOAL actionGoal;
  public GoalStatus goalStatus;
  public Time destructionTime;
  public ServerGoalHandle<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> goalHandle;

  public StatusTracker(
      DefaultActionServer<T_ACTION_FEEDBACK, T_ACTION_GOAL, T_ACTION_RESULT, T_FEEDBACK, T_GOAL, T_RESULT> server,
      GoalID goalID, byte status) {
    this.server = server;
    actionGoal = null;
    goalHandle = null;
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    goalStatus = messageFactory.newFromType(GoalStatus._TYPE);
    goalStatus.setGoalId(goalID);
    goalStatus.setStatus(status);
    destructionTime = new Time(0, 0);

  }

  public StatusTracker(T_ACTION_GOAL actionGoal, ActionSpec<?, ?, T_ACTION_GOAL, ?, ?, ?, ?> spec,
      GoalIDGenerator idGen) throws RosException {
    this.actionGoal = actionGoal;
    NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    goalHandle = null;
    goalStatus = messageFactory.newFromType(GoalStatus._TYPE);
    goalStatus.setGoalId(spec.getGoalIDFromActionGoal(actionGoal));
    goalStatus.setStatus(GoalStatus.PENDING);
    destructionTime = new Time(0, 0);

    if (goalStatus.getGoalId().getId() == null || goalStatus.getGoalId().getId().isEmpty()) {
      goalStatus.setGoalId(idGen.generateID());
    }

    if (goalStatus.getGoalId().getStamp().isZero()) {
      actionlib_msgs.GoalID gid = goalStatus.getGoalId();
      gid.setStamp(server.getNode().getCurrentTime());
      goalStatus.setGoalId(gid);
    }
  }

  public boolean isCancelRequestTracker() {
    return (actionGoal == null);
  }

}

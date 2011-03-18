// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_ACTIONLIB_H
#define OPENRAVE_ACTIONLIB_H

#include <rave/rave.h> // declare first

#include <cstdio>
#include <cstdlib>
#include <string>
#include <set>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/static_assert.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/format.hpp>

#include <ros/callback_queue.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_goal_state.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/client/terminal_state.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

using namespace OpenRAVE;
using namespace std;

namespace actionlib {
template <class ActionSpec>
class MyActionClient
{
private:
  ACTION_DEFINITION(ActionSpec);
  typedef ClientGoalHandle<ActionSpec> GoalHandleT;
  typedef MyActionClient<ActionSpec> MyActionClientT;

public:
  typedef boost::function<void (const SimpleClientGoalState& state,  const ResultConstPtr& result) > SimpleDoneCallback;
  typedef boost::function<void () > SimpleActiveCallback;
  typedef boost::function<void (const FeedbackConstPtr& feedback) > SimpleFeedbackCallback;

  /**
   * \brief Simple constructor
   *
   * Constructs a SingleGoalActionClient and sets up the necessary ros topics for the ActionInterface
   * \param name The action name. Defines the namespace in which the action communicates
   * \param spin_thread If true, spins up a thread to service this action's subscriptions. If false,
   *                    then the user has to call ros::spin() themselves. Defaults to True
   */
  MyActionClient(const std::string& name, bool spin_thread = true) : cur_simple_state_(SimpleGoalState::PENDING)
  {
    initSimpleClient(nh_, name, spin_thread);
  }

  /**
   * \brief Constructor with namespacing options
   *
   * Constructs a SingleGoalActionClient and sets up the necessary ros topics for
   * the ActionInterface, and namespaces them according the a specified NodeHandle
   * \param n The node handle on top of which we want to namespace our action
   * \param name The action name. Defines the namespace in which the action communicates
   * \param spin_thread If true, spins up a thread to service this action's subscriptions. If false,
   *                    then the user has to call ros::spin().
   */
  MyActionClient(ros::NodeHandle& n, const std::string& name, bool spin_thread = false) : cur_simple_state_(SimpleGoalState::PENDING)
  {
    initSimpleClient(n, name, spin_thread);

  }

  ~MyActionClient();

  /**
   * \brief Waits for the ActionServer to connect to this client
   *
   * Often, it can take a second for the action server & client to negotiate
   * a connection, thus, risking the first few goals to be dropped. This call lets
   * the user wait until the network connection to the server is negotiated
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the server connected in the allocated time. False on timeout
   */
  bool waitForServer(const ros::Duration& timeout = ros::Duration(0,0) ) { return ac_->waitForActionServerToStart(timeout); }

  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
   *
   * If a previous goal is already active when this is called. We simply forget
   * about that goal and start tracking the new goal. No cancel requests are made.
   * \param done_cb     Callback that gets called on transitions to Done
   * \param active_cb   Callback that gets called on transitions to Active
   * \param feedback_cb Callback that gets called whenever feedback for this goal is received
   */
  void sendGoal(const Goal& goal,
                SimpleDoneCallback     done_cb     = SimpleDoneCallback(),
                SimpleActiveCallback   active_cb   = SimpleActiveCallback(),
                SimpleFeedbackCallback feedback_cb = SimpleFeedbackCallback());

  /**
   * \brief Sends a goal to the ActionServer, and waits until the goal completes or a timeout is exceeded
   *
   * If the goal doesn't complete by the execute_timeout, then a preempt message is sent. This call
   * then waits up to the preempt_timeout for the goal to then finish.
   *
   * \param goal             The goal to be sent to the ActionServer
   * \param execute_timeout  Time to wait until a preempt is sent. 0 implies wait forever
   * \param preempt_timeout  Time to wait after a preempt is sent. 0 implies wait forever
   * \return The state of the goal when this call is completed
   */
  SimpleClientGoalState sendGoalAndWait(const Goal& goal,
                                        const ros::Duration& execute_timeout = ros::Duration(0,0),
                                        const ros::Duration& preempt_timeout = ros::Duration(0,0));

  /**
   * \brief Blocks until this goal finishes
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the goal finished. False if the goal didn't finish within the allocated timeout
   */
  bool waitForResult(const ros::Duration& timeout = ros::Duration(0,0) );

  /**
   * \brief Get the Result of the current goal
   * \return shared pointer to the result. Note that this pointer will NEVER be NULL
   */
  ResultConstPtr getResult();

  /**
   * \brief Get the state information for this goal
   *
   * Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
   * \return The goal's state. Returns LOST if this MyActionClient isn't tracking a goal.
   */
  SimpleClientGoalState getState();

  /**
   * \brief Cancel all goals currently running on the action server
   *
   * This preempts all goals running on the action server at the point that
   * this message is serviced by the ActionServer.
   */
  void cancelAllGoals();

  /**
   * \brief Cancel all goals that were stamped at and before the specified time
   * \param time All goals stamped at or before `time` will be canceled
   */
  void cancelGoalsAtAndBeforeTime(const ros::Time& time);

  /**
   * \brief Cancel the goal that we are currently pursuing
   */
  void cancelGoal();

  /**
   * \brief Stops tracking the state of the current goal. Unregisters this goal's callbacks
   *
   * This is useful if we want to make sure we stop calling our callbacks before sending a new goal.
   * Note that this does not cancel the goal, it simply stops looking for status info about this goal.
   */
  void stopTrackingGoal();

  bool isGoalExpired() { return gh_.isExpired(); }
private:
  typedef ActionClient<ActionSpec> ActionClientT;
  ros::NodeHandle nh_;
  GoalHandleT gh_;

  SimpleGoalState cur_simple_state_;

  // Signalling Stuff
  boost::condition done_condition_;
  boost::mutex done_mutex_;

  // User Callbacks
  SimpleDoneCallback done_cb_;
  SimpleActiveCallback active_cb_;
  SimpleFeedbackCallback feedback_cb_;

  // Spin Thread Stuff
  boost::mutex terminate_mutex_;
  bool need_to_terminate_;
  boost::thread* spin_thread_;
  ros::CallbackQueue callback_queue;

  boost::scoped_ptr<ActionClientT> ac_;  // Action client depends on callback_queue, so it must be destroyed before callback_queue

  // ***** Private Funcs *****
  void initSimpleClient(ros::NodeHandle& n, const std::string& name, bool spin_thread);
  void handleTransition(GoalHandleT gh);
  void handleFeedback(GoalHandleT gh, const FeedbackConstPtr& feedback);
  void setSimpleState(const SimpleGoalState::StateEnum& next_state);
  void setSimpleState(const SimpleGoalState& next_state);
  void spinThread();
};



template<class ActionSpec>
void MyActionClient<ActionSpec>::initSimpleClient(ros::NodeHandle& n, const std::string& name, bool spin_thread)
{
  if (spin_thread)
  {
    ROS_DEBUG("Spinning up a thread for the MyActionClient");
    need_to_terminate_ = false;
    spin_thread_ = new boost::thread(boost::bind(&MyActionClient<ActionSpec>::spinThread, this));
    ac_.reset(new ActionClientT(n, name, &callback_queue));
  }
  else
  {
    spin_thread_ = NULL;
    ac_.reset(new ActionClientT(n, name));
  }
}

template<class ActionSpec>
MyActionClient<ActionSpec>::~MyActionClient()
{
  if (spin_thread_)
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      need_to_terminate_ = true;
    }
    spin_thread_->join();
    delete spin_thread_;
  }
  gh_.reset();
  ac_.reset();
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::spinThread()
{
  while (nh_.ok())
  {
    {
      boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
      if (need_to_terminate_)
        break;
    }
    callback_queue.callAvailable(ros::WallDuration(0.1f));
  }
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::setSimpleState(const SimpleGoalState::StateEnum& next_state)
{
  setSimpleState( SimpleGoalState(next_state) );
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::setSimpleState(const SimpleGoalState& next_state)
{
  ROS_DEBUG("Transitioning SimpleState from [%s] to [%s]",
            cur_simple_state_.toString().c_str(),
            next_state.toString().c_str());
  cur_simple_state_ = next_state;
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::sendGoal(const Goal& goal,
                                              SimpleDoneCallback     done_cb,
                                              SimpleActiveCallback   active_cb,
                                              SimpleFeedbackCallback feedback_cb)
{
  // Reset the old GoalHandle, so that our callbacks won't get called anymore
  gh_.reset();

  // Store all the callbacks
  done_cb_     = done_cb;
  active_cb_   = active_cb;
  feedback_cb_ = feedback_cb;

  cur_simple_state_ = SimpleGoalState::PENDING;

  // Send the goal to the ActionServer
  gh_ = ac_->sendGoal(goal, boost::bind(&MyActionClientT::handleTransition, this, _1),
                            boost::bind(&MyActionClientT::handleFeedback, this, _1, _2));
}

template<class ActionSpec>
SimpleClientGoalState MyActionClient<ActionSpec>::getState()
{
  if (gh_.isExpired())
  {
    ROS_ERROR("Trying to getState() when no goal is running. You are incorrectly using MyActionClient");
    return SimpleClientGoalState(SimpleClientGoalState::LOST);
  }

  CommState comm_state_ = gh_.getCommState();

  switch( comm_state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
    case CommState::PENDING:
    case CommState::RECALLING:
      return SimpleClientGoalState(SimpleClientGoalState::PENDING);
    case CommState::ACTIVE:
    case CommState::PREEMPTING:
      return SimpleClientGoalState(SimpleClientGoalState::ACTIVE);
    case CommState::DONE:
    {
      switch(gh_.getTerminalState().state_)
      {
        case TerminalState::RECALLED:
          return SimpleClientGoalState(SimpleClientGoalState::RECALLED, gh_.getTerminalState().text_);
        case TerminalState::REJECTED:
          return SimpleClientGoalState(SimpleClientGoalState::REJECTED, gh_.getTerminalState().text_);
        case TerminalState::PREEMPTED:
          return SimpleClientGoalState(SimpleClientGoalState::PREEMPTED, gh_.getTerminalState().text_);
        case TerminalState::ABORTED:
          return SimpleClientGoalState(SimpleClientGoalState::ABORTED, gh_.getTerminalState().text_);
        case TerminalState::SUCCEEDED:
          return SimpleClientGoalState(SimpleClientGoalState::SUCCEEDED, gh_.getTerminalState().text_);
        case TerminalState::LOST:
          return SimpleClientGoalState(SimpleClientGoalState::LOST, gh_.getTerminalState().text_);
        default:
          ROS_ERROR("Unknown terminal state [%u]. This is a bug in MyActionClient", gh_.getTerminalState().state_);
          return SimpleClientGoalState(SimpleClientGoalState::LOST, gh_.getTerminalState().text_);
      }
    }
    case CommState::WAITING_FOR_RESULT:
    case CommState::WAITING_FOR_CANCEL_ACK:
    {
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
          return SimpleClientGoalState(SimpleClientGoalState::PENDING);
        case SimpleGoalState::ACTIVE:
          return SimpleClientGoalState(SimpleClientGoalState::ACTIVE);
        case SimpleGoalState::DONE:
          ROS_ERROR("In WAITING_FOR_RESULT or WAITING_FOR_CANCEL_ACK, yet we are in SimpleGoalState DONE. This is a bug in MyActionClient");
          return SimpleClientGoalState(SimpleClientGoalState::LOST);
        default:
          ROS_ERROR("Got a SimpleGoalState of [%u]. This is a bug in MyActionClient", cur_simple_state_.state_);
      }
    }
    default:
      break;
  }
  ROS_ERROR("Error trying to interpret CommState - %u", comm_state_.state_);
  return SimpleClientGoalState(SimpleClientGoalState::LOST);
}

template<class ActionSpec>
typename MyActionClient<ActionSpec>::ResultConstPtr MyActionClient<ActionSpec>::getResult()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to getResult() when no goal is running. You are incorrectly using MyActionClient");

  if (gh_.getResult())
    return gh_.getResult();

  return ResultConstPtr(new Result);
}


template<class ActionSpec>
void MyActionClient<ActionSpec>::cancelAllGoals()
{
  ac_->cancelAllGoals();
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::cancelGoalsAtAndBeforeTime(const ros::Time& time)
{
  ac_->cancelAllGoalsBeforeTime(time);
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::cancelGoal()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to cancelGoal() when no goal is running. You are incorrectly using MyActionClient");

  gh_.cancel();
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::stopTrackingGoal()
{
  if (gh_.isExpired())
    ROS_ERROR("Trying to stopTrackingGoal() when no goal is running. You are incorrectly using MyActionClient");
  gh_.reset();
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::handleFeedback(GoalHandleT gh, const FeedbackConstPtr& feedback)
{
  if (gh_ != gh)
    ROS_ERROR("Got a callback on a goalHandle that we're not tracking.  \
               This is an internal MyActionClient/ActionClient bug.  \
               This could also be a GoalID collision");
  if (feedback_cb_)
    feedback_cb_(feedback);
}

template<class ActionSpec>
void MyActionClient<ActionSpec>::handleTransition(GoalHandleT gh)
{
  CommState comm_state_ = gh.getCommState();
  switch (comm_state_.state_)
  {
    case CommState::WAITING_FOR_GOAL_ACK:
      ROS_ERROR("BUG: Shouldn't ever get a transition callback for WAITING_FOR_GOAL_ACK");
      break;
    case CommState::PENDING:
      ROS_ERROR_COND( cur_simple_state_ != SimpleGoalState::PENDING,
                      "BUG: Got a transition to CommState [%s] when our in SimpleGoalState [%s]",
                      comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
      break;
    case CommState::ACTIVE:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
          setSimpleState(SimpleGoalState::ACTIVE);
          if (active_cb_)
            active_cb_();
          break;
        case SimpleGoalState::ACTIVE:
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a transition to CommState [%s] when in SimpleGoalState [%s]",
                    comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    case CommState::WAITING_FOR_RESULT:
      break;
    case CommState::WAITING_FOR_CANCEL_ACK:
      break;
    case CommState::RECALLING:
      ROS_ERROR_COND( cur_simple_state_ != SimpleGoalState::PENDING,
                      "BUG: Got a transition to CommState [%s] when our in SimpleGoalState [%s]",
                      comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
      break;
    case CommState::PREEMPTING:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
          setSimpleState(SimpleGoalState::ACTIVE);
          if (active_cb_)
            active_cb_();
          break;
        case SimpleGoalState::ACTIVE:
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a transition to CommState [%s] when in SimpleGoalState [%s]",
                     comm_state_.toString().c_str(), cur_simple_state_.toString().c_str());
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    case CommState::DONE:
      switch (cur_simple_state_.state_)
      {
        case SimpleGoalState::PENDING:
        case SimpleGoalState::ACTIVE:
          done_mutex_.lock();
          setSimpleState(SimpleGoalState::DONE);
          done_mutex_.unlock();

          if (done_cb_)
            done_cb_(getState(), gh.getResult());

          done_condition_.notify_all();
          break;
        case SimpleGoalState::DONE:
          ROS_ERROR("BUG: Got a second transition to DONE");
          break;
        default:
          ROS_FATAL("Unknown SimpleGoalState %u", cur_simple_state_.state_);
          break;
      }
      break;
    default:
      ROS_ERROR("Unknown CommState received [%u]", comm_state_.state_);
      break;
  }
}

template<class ActionSpec>
bool MyActionClient<ActionSpec>::waitForResult(const ros::Duration& timeout )
{
  if (gh_.isExpired())
  {
    ROS_ERROR("Trying to waitForGoalToFinish() when no goal is running. You are incorrectly using MyActionClient");
    return false;
  }

  if (timeout < ros::Duration(0,0))
    ROS_WARN("Timeouts can't be negative. Timeout is [%.2fs]", timeout.toSec());

  ros::Time timeout_time = ros::Time::now() + timeout;

  boost::mutex::scoped_lock lock(done_mutex_);

  // Hardcode how often we check for node.ok()
  ros::Duration loop_period = ros::Duration().fromSec(.1);

  while (nh_.ok())
  {
    // Determine how long we should wait
    ros::Duration time_left = timeout_time - ros::Time::now();

    // Check if we're past the timeout time
    if (timeout > ros::Duration(0,0) && time_left <= ros::Duration(0,0) )
      break;

    if (cur_simple_state_ == SimpleGoalState::DONE)
      break;

    // Truncate the time left
    if (time_left > loop_period || timeout == ros::Duration())
      time_left = loop_period;

    done_condition_.timed_wait(lock, boost::posix_time::milliseconds(time_left.toSec() * 1000.0f));
  }

  return (cur_simple_state_ == SimpleGoalState::DONE);
}

template<class ActionSpec>
SimpleClientGoalState MyActionClient<ActionSpec>::sendGoalAndWait(const Goal& goal,
                                                                      const ros::Duration& execute_timeout,
                                                                      const ros::Duration& preempt_timeout)
{
  sendGoal(goal);

  // See if the goal finishes in time
  if (waitForResult(execute_timeout))
  {
    ROS_DEBUG("Goal finished within specified execute_timeout [%.2f]", execute_timeout.toSec());
    return getState();
  }

  ROS_DEBUG("Goal finished within specified execute_timeout [%.2f]", execute_timeout.toSec());

  // It didn't finish in time, so we need to preempt it
  cancelGoal();

  // Now wait again and see if it finishes
  if (waitForResult(preempt_timeout))
    ROS_DEBUG("Preempt finished within specified preempt_timeout [%.2f]", preempt_timeout.toSec());
  else
    ROS_DEBUG("Preempt didn't finish specified preempt_timeout [%.2f]", preempt_timeout.toSec());
  return getState();
}

}

class ROSActionLibController : public ControllerBase
{
public:
 ROSActionLibController(EnvironmentBasePtr penv, std::istream& ss) : ControllerBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nAllows OpenRAVE to control a robot through the ROS actionlib interface.";
        _bDestroyThread = true;
        _nControlTransformation = 0;
        _jointstatetopic="/joint_states";
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss ) {
                break;
            }

            if( cmd == "jointstate") 
                ss >> _jointstatetopic;
            else if( cmd == "state")
                ss >> _controllerstatetopic;
            else if( cmd == "action" )
                ss >> _actiontopic;
            else
                break;
        }
    }
    virtual ~ROSActionLibController() {
        _Destroy();
    }
    
    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _Destroy();
        _probot = robot;
        _dofindices = dofindices;
        _nControlTransformation = nControlTransformation;

        int argc=0;
        ros::init(argc,NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
        if( !ros::master::check() ) {
            return false;
        }
        
        _node.reset(new ros::NodeHandle());
        _bDestroyThread = false;
        _threadros = boost::thread(boost::bind(&ROSActionLibController::_threadrosfn, this));

        _subjointstate = _node->subscribe(_jointstatetopic, 10, &ROSActionLibController::_jointstatecb, this);
        if( _controllerstatetopic.size() > 0 )
            _subcontrollerstate = _node->subscribe(_controllerstatetopic,1,&ROSActionLibController::_controllerstatecb, this);
        else {
            RAVELOG_WARN("no controller state topic, filling map joints with default\n");
            FOREACHC(itjoint,_probot->GetJoints()) {
                _vjointnames.push_back(make_pair((*itjoint)->GetName(), (*itjoint)->GetJointIndex()));
            }
        }

        _probot->GetDOFValues(_vlastjointvalues);
        _vlastjointvel.resize(0);
        _vlastjointvel.resize(_vlastjointvalues.size(),0.0f);
        _vlastjointtorque = _vlastjointvel;

        return true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
    virtual int IsControlTransformation() const { return _nControlTransformation; }

    virtual void Reset(int options)
    {
        if( !!_ac ) {
            _ac->cancelAllGoals();
            _bHasGoal = false;
        }
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        TrajectoryBasePtr ptraj;
        {
            EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
            ptraj = RaveCreateTrajectory(GetEnv(),_probot->GetDOF());
            vector<dReal> vcurrentvalues;
            _probot->GetDOFValues(vcurrentvalues);
            ptraj->AddPoint(TrajectoryBase::TPOINT(vcurrentvalues,0));
            ptraj->AddPoint(TrajectoryBase::TPOINT(values,0));
            ptraj->CalcTrajTiming(_probot,TrajectoryBase::LINEAR,true,false);
        }
        
        return SetPath(ptraj);
    }
    
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( ptraj->GetTotalDuration() <= 0 || ptraj->GetPoints().size() == 0 )
            return false;
        boost::mutex::scoped_lock lock(_mutex);
        if( _vjointnames.size() == 0 ) {
            RAVELOG_INFO("joint names not initialized\n");
            return false;
        }
        _checkaction();
        if( !_ac )
            return false;
        _ac->cancelAllGoals();
        pr2_controllers_msgs::JointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2); // start in 100ms
        goal.trajectory.joint_names.reserve(_vjointnames.size());
        vector<int> vindices;
        FOREACH(itname,_vjointnames) {
            goal.trajectory.joint_names.push_back(itname->first);
            BOOST_ASSERT(itname->second<ptraj->GetDOF());
            vindices.push_back(itname->second);
        }
        FOREACHC(itpoint,ptraj->GetPoints()) {
            trajectory_msgs::JointTrajectoryPoint pt;
            pt.positions.reserve(vindices.size());
            pt.velocities.reserve(vindices.size());
            FOREACH(it,vindices) {
                pt.positions.push_back(itpoint->q[*it]);
                pt.velocities.push_back(itpoint->qdot[*it]);
            }
            pt.time_from_start.fromSec((double)itpoint->time);
            goal.trajectory.points.push_back(pt);
        }

        _ac->sendGoal(goal);
        _bHasGoal = true;
        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
    }

    virtual bool IsDone()
    {
        if( !_bHasGoal || !_ac )
            return true;
        if( _ac->isGoalExpired() )
            return true;
        if( !_ac->waitForResult(ros::Duration(0.0001)) )
            return false;
        return _ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    virtual dReal GetTime() const
    {
        return !_ac ? 0 : (ros::Time::now()-_actiontime).toSec();
    }

    virtual void GetVelocity(std::vector<dReal>& vel) const {
        boost::mutex::scoped_lock lock(_mutex);
        vel = _vlastjointvel;
    }

    virtual void GetTorque(std::vector<dReal>& torque) const {
        boost::mutex::scoped_lock lock(_mutex);
        torque = _vlastjointtorque;
    }
    
    virtual RobotBasePtr GetRobot() const { return _probot; }

protected:
    virtual void _Destroy()
    {
        if( !!_ac ) {
            _ac->cancelAllGoals();
            _ac.reset();
            _bHasGoal = false;
        }

        _bDestroyThread = true;
        _node.reset();
        _subjointstate.shutdown();
        _subcontrollerstate.shutdown();
        _threadros.join(); // deadlock condition with environment
        _probot.reset();
        _vjointnames.clear();
    }
    
    virtual void _jointstatecb(const sensor_msgs::JointStateConstPtr& jstate)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( _bDestroyThread )
            return;
        EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        _probot->GetDOFValues(_vlastjointvalues);
        //resize the vel and torque vectors if necessary:
        _vlastjointvel.resize(_vlastjointvalues.size(),0.0f);
        _vlastjointtorque.resize(_vlastjointvalues.size(),0.0f);
        for(size_t i = 0; i < jstate->name.size(); ++i) {
            KinBody::JointPtr pjoint = _probot->GetJoint(jstate->name[i]);
            if( !pjoint ) {
                RAVELOG_VERBOSE(str(boost::format("could not find joint %s\n")%jstate->name[i]));
                continue;
            }
            else {
                _vlastjointvalues.at(pjoint->GetJointIndex()) = jstate->position.at(i);
                _vlastjointvel.at(pjoint->GetJointIndex()) = jstate->velocity.at(i);
                _vlastjointtorque.at(pjoint->GetJointIndex()) = jstate->effort.at(i);
            }
        }
        _probot->SetJointValues(_vlastjointvalues);
    }

    virtual void _controllerstatecb(const pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr& cstate)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( _bDestroyThread )
            return;
        _vjointnames.clear();
        FOREACH(itname,cstate->joint_names) {
            KinBody::JointPtr pjoint = _probot->GetJoint(*itname);
            _vjointnames.push_back(make_pair(*itname, pjoint->GetJointIndex()));
        }
    }
    
    virtual void _threadrosfn()
    {
        while(!_bDestroyThread && ros::ok()) {
            ros::spinOnce();
            usleep(1000); // query every 1ms?
        }
    }

    virtual void _checkaction()
    {
        if( _actiontopic.size() == 0 )
            return;
        
        if( !!_ac ) {
            // check if still valid
            return;
        }
        
        _ac.reset(new actionlib::MyActionClient<pr2_controllers_msgs::JointTrajectoryAction>(_actiontopic,true));
        ros::Time start = ros::Time::now();
        if( !_ac->waitForServer(ros::Duration(10.0)) ) {
            _ac.reset();
            RAVELOG_WARN(str(boost::format("failed to connect to action %s\n")%_actiontopic));
        }
    }

    bool _bDestroyThread;
    RobotBasePtr _probot;           ///< controlled body
    boost::shared_ptr<actionlib::MyActionClient<pr2_controllers_msgs::JointTrajectoryAction> > _ac;
    boost::shared_ptr<ros::NodeHandle> _node;
    boost::thread _threadros;
    string _actiontopic;
    pr2_controllers_msgs::JointTrajectoryControllerState _cstate;
    ros::Subscriber _subjointstate, _subcontrollerstate;
    ros::Time _actiontime;
    vector<dReal> _vlastjointvalues, _vlastjointvel, _vlastjointtorque;
    vector< pair<string,int> > _vjointnames;
    mutable boost::mutex _mutex;
    bool _bHasGoal;
    std::vector<int> _dofindices;
    int _nControlTransformation;
    string _jointstatetopic, _controllerstatetopic;
};

#endif

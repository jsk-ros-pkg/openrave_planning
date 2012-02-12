// Copyright (C) 2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
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
#include <ros/node_handle.h>
#include <ros/master.h>

#include <vector>
#include <string>
#include <stdexcept>

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/format.hpp>

#include <openrave-core.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)

using namespace std;
using namespace OpenRAVE;

class RobotFramePublisher
{
public:
    RobotFramePublisher(const string& robotfile)
    {
        if( robotfile.size() == 0 )
            throw runtime_error("bad robot file");

        // create the main environment
        _penv = OpenRAVE::RaveCreateEnvironment();
        if( !_penv ) {
            throw runtime_error("failed to create openrave environment");
        }
        _probot = _penv->ReadRobotXMLFile(RobotBasePtr(), robotfile, std::list<std::pair<std::string,std::string> >());
        if( !_probot ) {
            throw runtime_error("failed to create openrave robot");
        }
        _penv->AddRobot(_probot);

        RAVELOG_INFOA(str(boost::format("found openrave robot %s, numjoints=%d\n")%_probot->GetName()%_probot->GetDOF()));

        FOREACH(itlink, _probot->GetLinks())
        _vlinknames.push_back((*itlink)->GetName());
        FOREACH(itjoint, _probot->GetJoints())
        _vjointnames.push_back((*itjoint)->GetName());
        _probot->GetDOFValues(_vjointvalues);

        set<int> setusedlinks;
        // go through all dummy joints and prune more links
        int i = 0;
        while(i < _probot->GetDOF()) {
            KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(i);
            bool bDummy = true;
            if( pjoint->GetDOF() > 0 ) {
                vector<dReal> vlower, vupper;
                pjoint->GetLimits(vlower, vupper);
                for(int j=0; j<pjoint->GetDOF(); ++j) {
                    if( vlower[j] != vupper[j] ) {
                        bDummy = false;
                        break;
                    }
                }
            }

            if( bDummy ) {
                bool bHasFirst = setusedlinks.find(pjoint->GetFirstAttached()->GetIndex()) != setusedlinks.end();
                bool bHasSecond = setusedlinks.find(pjoint->GetSecondAttached()->GetIndex()) != setusedlinks.end();
                if( !bHasFirst || !bHasSecond ) {
                    if( _probot->DoesAffect(pjoint->GetJointIndex(), pjoint->GetFirstAttached()->GetIndex()) )
                        // first link is child link
                        _vStaticTransforms.push_back(boost::make_tuple(getBtTransform(pjoint->GetSecondAttached()->GetTransform().inverse() * pjoint->GetFirstAttached()->GetTransform()), _vlinknames[pjoint->GetSecondAttached()->GetIndex()], _vlinknames[pjoint->GetFirstAttached()->GetIndex()]));
                    else
                        _vStaticTransforms.push_back(boost::make_tuple(getBtTransform(pjoint->GetFirstAttached()->GetTransform().inverse() * pjoint->GetSecondAttached()->GetTransform()), _vlinknames[pjoint->GetFirstAttached()->GetIndex()], _vlinknames[pjoint->GetSecondAttached()->GetIndex()]));

                    setusedlinks.insert(pjoint->GetFirstAttached()->GetIndex());
                    setusedlinks.insert(pjoint->GetSecondAttached()->GetIndex());
                    i = 0;
                    continue;
                }
            }

            ++i;
        }

        // publish sensor frames
        FOREACH(itsensor, _probot->GetAttachedSensors()) {
            // only send sensor data if name is valid
            if( (*itsensor)->GetName().size() > 0 ) {
                RAVELOG_INFO(str(boost::format("publishing sensor transform %s\n")%(*itsensor)->GetName()));
                _vStaticTransforms.push_back(boost::make_tuple(getBtTransform((*itsensor)->GetRelativeTransform()),_vlinknames[(*itsensor)->GetAttachingLink()->GetIndex()], (*itsensor)->GetName()));
            }
        }

        // publish manipulator frames
        FOREACH(itmanip, _probot->GetManipulators()) {
            // only send sensor data if name is valid
            if( (*itmanip)->GetName().size() > 0 ) {
                RAVELOG_INFO(str(boost::format("publishing manipulator transform %s\n")%(*itmanip)->GetName()));
                _vStaticTransforms.push_back(boost::make_tuple(getBtTransform((*itmanip)->GetLocalToolTransform()),_vlinknames[(*itmanip)->GetEndEffector()->GetIndex()], (*itmanip)->GetName()));
            }
        }

        _node.reset(new ros::NodeHandle());
        _tfbroadcaster.reset(new tf::TransformBroadcaster());

        char str[32];
        int index = 0;
        while(1) {
            string statetopic;
            sprintf(str,"robot%d",index++);
            if( !_node->getParam(str,statetopic) ) {
                if( index > 10 )
                    break;
                continue;
            }

            _listsubscriptions.push_back(_node->subscribe(statetopic, 2, &RobotFramePublisher::statecb, this));
        }

        if( _listsubscriptions.size() == 0 ) {
            throw runtime_error("no robot mechanism states attached, define with robotX ROS parameters");
        }
    }

    virtual ~RobotFramePublisher()
    {
        if( !!_probot && !!_penv )
            _penv->Remove(_probot);
        _probot.reset();
        if( !!_penv )
            _penv->Destroy();
        _penv.reset();
        _tfbroadcaster.reset();
        _listsubscriptions.clear();
        _node.reset();
    }

    void statecb(const sensor_msgs::JointStateConstPtr& pmstate)
    {
        EnvironmentMutex::scoped_lock lock(_penv->GetMutex());

        list<int> listjoints;
        for(size_t i = 0; i < pmstate->name.size(); ++i) {
            vector<string>::iterator itname = find(_vjointnames.begin(),_vjointnames.end(),pmstate->name.at(i));
            if( itname == _vjointnames.end() ) {
                RAVELOG_WARN("joint %s not in openrave robot\n",pmstate->name.at(i).c_str());
                continue;
            }

            _vjointvalues[itname-_vjointnames.begin()] = pmstate->position.at(i);
            listjoints.push_back(itname-_vjointnames.begin());
        }

        if( _vjointvalues.size() > 0 )
            _probot->SetJointValues(_vjointvalues);

        // publish joints
        FOREACH(it,listjoints) {
            KinBody::JointPtr pjoint = _probot->GetJoints().at(*it);
            KinBody::LinkPtr pchild = pjoint->GetFirstAttached();
            KinBody::LinkPtr pparent = pjoint->GetSecondAttached();

            if( !pchild )
                swap(pchild,pparent);
            else if( !!pparent ) {
                // both links valid
                if( _probot->DoesAffect(*it, pjoint->GetSecondAttached()->GetIndex()) )
                    swap(pchild,pparent);                                             // second link is child link
            }

            Transform tchild, tparent;
            if( !!pchild )
                tchild = pchild->GetTransform();
            if( !!pparent )
                tparent = pparent->GetTransform();
            _tfbroadcaster->sendTransform(tf::StampedTransform(getBtTransform(tparent.inverse() * tchild), pmstate->header.stamp, _vlinknames[pparent->GetIndex()], _vlinknames[pchild->GetIndex()]));
        }

        // publish passive joints
        std::vector<int> vmimicdofs;
        FOREACH(itpassive, _probot->GetPassiveJoints()) {
            FOREACH(it,listjoints) {
                (*itpassive)->GetMimicDOFIndices(vmimicdofs,0);
                if( find(vmimicdofs.begin(),vmimicdofs.end(),*it) != vmimicdofs.end() ) {
                    KinBody::LinkPtr pchild = (*itpassive)->GetFirstAttached();
                    KinBody::LinkPtr pparent = (*itpassive)->GetSecondAttached();
                    if( !pchild ) {
                        swap(pchild,pparent);
                    }
                    else if( !!pparent ) {
                        // both links valid
                        if( _probot->DoesAffect(*it, (*itpassive)->GetSecondAttached()->GetIndex()) ) {
                            swap(pchild,pparent);  // second link is child link
                        }
                    }

                    Transform tchild, tparent;
                    if( !!pchild )
                        tchild = pchild->GetTransform();
                    if( !!pparent )
                        tparent = pparent->GetTransform();
                    _tfbroadcaster->sendTransform(tf::StampedTransform(getBtTransform(tparent.inverse() * tchild), pmstate->header.stamp, _vlinknames[pparent->GetIndex()], _vlinknames[pchild->GetIndex()]));
                    break;
                }
            }
        }

        // publish static frames
        FOREACH(itstatic,_vStaticTransforms)
        _tfbroadcaster->sendTransform(tf::StampedTransform(boost::get<0>(*itstatic), pmstate->header.stamp, boost::get<1>(*itstatic), boost::get<2>(*itstatic)));
    }

    btTransform getBtTransform(const Transform& t)
    {
        return btTransform(btQuaternion(t.rot.y,t.rot.z,t.rot.w,t.rot.x), btVector3(t.trans.x,t.trans.y,t.trans.z));
    }

protected:
    boost::shared_ptr<ros::NodeHandle> _node;
    list<ros::Subscriber> _listsubscriptions;
    boost::shared_ptr<tf::TransformBroadcaster> _tfbroadcaster;
    boost::shared_ptr<EnvironmentBase> _penv;
    boost::shared_ptr<RobotBase> _probot;
    vector<string> _vlinknames, _vjointnames;
    vector<dReal> _vjointvalues;
    vector<boost::tuple<btTransform,string,string> > _vStaticTransforms; ///< relative transforms of links to publish with tf, first link is child, second is parent
};


int main(int argc, char ** argv)
{
    // parse the command line options
    string robotname;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("tfsender [--robotfile openravefile]\n"
                   "  Start a node that connects to a mechanism_state message and publishes tf frames for the robot links\n");

            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
            robotname = argv[i+1];
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv,"openrave_tfsender");
    if( !ros::master::check() ) {
        return 1;
    }
    RaveInitialize(true);
    boost::shared_ptr<RobotFramePublisher> ppublisher(new RobotFramePublisher(robotname));
    ros::spin();

    ppublisher.reset();
    return 0;
}

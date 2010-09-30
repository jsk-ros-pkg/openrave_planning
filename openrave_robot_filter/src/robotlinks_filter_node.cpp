// Software License Agreement (BSD License)
// Copyright (c) 2008-2010, Rosen Diankov (rdiankov@cs.cmu.edu), Willow Garage, Inc.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * The name of the author may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
/// \author Rosen Diankov (rdiankov@cs.cmu.edu)
#include <openrave-core.h>

#include <vector>
#include <sstream>
#include <cstdio>

#include <ros/node_handle.h>
#include <ros/master.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/format.hpp>

#include <cstdio>
#include <cstdlib>

extern "C"
{
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}

using namespace OpenRAVE;
using namespace std;

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)

class RobotLinksFilter
{
    struct LINK
    {
        string tfframe;
        vector<Vector> vconvexhull; ///< convex hull of the link
        Transform tstart, tend;
        vector<Vector> vnewconvexhull;
    };
    struct LASERPOINT
    {
        LASERPOINT() {}
        LASERPOINT(const Vector& ptnew, dReal timenew) : pt(ptnew), time(timenew),inside(0) {}
        Vector pt;
        dReal time; // 0-1 value specifying (stamp-stampstart)/(stampend-stampstart) where stamp is time of point,
                    // startstart and startend are start and end times of the entire scan
        int inside; // inside robot frame
    };

    tf::TransformListener                                 _tf;
    ros::NodeHandle                                       _nh, _root_handle;
    boost::shared_ptr< tf::MessageFilter<sensor_msgs::PointCloud> > _mn;
    boost::shared_ptr< message_filters::Subscriber<sensor_msgs::PointCloud> > _sub;
    std::string _sensor_frame;
    ros::Publisher                                        _pointCloudPublisher;
    ros::Subscriber                                       _no_filter_sub;

    vector<LINK> _vLinkHulls;
    boost::shared_ptr<ros::NodeHandle> _ros;
    vector<LASERPOINT> _vlaserpoints;
    sensor_msgs::PointCloud _pointcloudout;
    string _robotname;
    double _convexpadding, _min_sensor_dist;
    bool _bAccurateTiming; ///< if true, will interpolate the convex hulls for every time stamp

public:
    RobotLinksFilter(const string& robotname) : _nh("~"), _robotname(robotname), _convexpadding(0.01), _bAccurateTiming(false)
    {
        _nh.param<std::string>("sensor_frame", _sensor_frame, std::string());
        _nh.param<double>("min_sensor_dist", _min_sensor_dist, 0.01);;
        _nh.param<double>("self_see_default_padding", _convexpadding, .01);
        //_nh.param<double>("self_see_default_scale", default_scale, 1.0);

        if( InitRobotLinksFromOpenRAVE(robotname) ) {
    
            _sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud>(_root_handle, "cloud_in", 1));
            _mn.reset(new tf::MessageFilter<sensor_msgs::PointCloud>(*_sub, _tf, "", 2));

            //_mn = new tf::MessageNotifier<sensor_msgs::PointCloud>(tf_, boost::bind(&SelfFilter::cloudCallback, this, _1), "cloud_in", "", 1);
            _pointCloudPublisher = _root_handle.advertise<sensor_msgs::PointCloud>("cloud_out", 1);

            vector<string> frames;
            FOREACH(itlink,_vLinkHulls)
                frames.push_back(itlink->tfframe);
            _mn->setTargetFrames(frames);
            _mn->registerCallback(boost::bind(&RobotLinksFilter::PointCloudCallback, this, _1));
        }
        else {
            RAVELOG_ERROR(str(boost::format("failed to init robot %s\n")%robotname));
            _no_filter_sub = _root_handle.subscribe<sensor_msgs::PointCloud>("cloud_in", 1, boost::bind(&RobotLinksFilter::noFilterCallback, this, _1));
        }
        RAVELOG_INFO(str(boost::format("self collision with robot %s, padding=%f\n")%_robotname%_convexpadding));
    }
    virtual ~RobotLinksFilter()
    {
        _mn.reset();
        _sub.reset();
        _no_filter_sub.shutdown();
    }

    bool InitRobotLinksFromOpenRAVE(const string& robotname)
    {
        _vLinkHulls.clear();

        if( robotname.size() == 0 )
            return false;

        RAVELOG_INFO("opening OpenRAVE robot file %s\n", robotname.c_str());
        
        // create the main environment
        EnvironmentBasePtr penv = RaveCreateEnvironment();
        if( !penv ) {
            return false;
        }
        // load the scene
        if( !penv->Load(robotname) ) {
            RAVELOG_ERROR("RobotLinksFilter failed create robot %s\n", robotname.c_str());
            return false;
        }

        // get the first robot
        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        if( vrobots.size() == 0 ) {
            RAVELOG_ERROR("RobotLinksFilter no robots in file %s\n", robotname.c_str());
            return false;
        }
        
        RobotBasePtr probot = vrobots.front();    
        RAVELOG_INFOA(str(boost::format("generating convex hulls for robot %s, num links: %d")%probot->GetName()%probot->GetLinks().size()));

        ros::Time starthull = ros::Time::now();
        _vLinkHulls.resize(probot->GetLinks().size());
        vector<LINK>::iterator ithull = _vLinkHulls.begin();
        size_t totalplanes = 0;
        FOREACH(itlink, probot->GetLinks()) {
            // compute convex hull
            if( compute_convex_hull((*itlink)->GetCollisionData().vertices, ithull->vconvexhull) ) {
                totalplanes += ithull->vconvexhull.size();
                RAVELOG_DEBUG(str(boost::format("link %s convex hull has %d planes\n")%(*itlink)->GetName()%ithull->vconvexhull.size()));
            }
            else
                RAVELOG_ERROR(str(boost::format("failed to compute convex hull for link %s\n")%(*itlink)->GetName()));

            ithull->tfframe = (*itlink)->GetName();
            ++ithull;
        }

        RAVELOG_INFOA(str(boost::format("total convex planes: %d, time: %fs")%totalplanes%(ros::Time::now()-starthull).toSec()));
        return true;
    }

private:

    void noFilterCallback(const sensor_msgs::PointCloudConstPtr &cloud){
        _pointCloudPublisher.publish(cloud);
        ROS_DEBUG("Self filter publishing unfiltered frame");
    }

    void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& _pointcloudin)
    {
        if( _vLinkHulls.size() == 0 ) {
            // just pass the data
            _pointCloudPublisher.publish(_pointcloudin);
            return;
        }

        if( _pointcloudin->points.size() == 0 )
            return;

        ros::WallTime tm = ros::WallTime::now();

        if( _bAccurateTiming )
            PruneWithAccurateTiming(*_pointcloudin, _vlaserpoints);
        else
            PruneWithSimpleTiming(*_pointcloudin, _vlaserpoints);

        int totalpoints = 0;
        FOREACH(itpoint, _vlaserpoints)
            totalpoints += itpoint->inside==0;

        _pointcloudout.header = _pointcloudin->header;
        _pointcloudout.points.resize(totalpoints);
        _pointcloudout.channels.resize(_pointcloudin->channels.size());
        for(int ichan = 0; ichan < (int)_pointcloudin->channels.size(); ++ichan) {
            _pointcloudout.channels[ichan].name = _pointcloudin->channels[ichan].name;
            _pointcloudout.channels[ichan].values.resize(totalpoints);
        }

        for(int oldindex = 0, newindex = 0; oldindex < (int)_vlaserpoints.size(); ++oldindex) {
            if( _vlaserpoints[oldindex].inside )
                continue;

            _pointcloudout.points[newindex] = _pointcloudin->points[oldindex];
            for(int ichan = 0; ichan < (int)_pointcloudin->channels.size(); ++ichan)
                _pointcloudout.channels[ichan].values[newindex] = _pointcloudin->channels[ichan].values[oldindex];
            ++newindex;
        }

        RAVELOG_DEBUG("robotlinks_filter_node published %d points, processing time=%fs\n", totalpoints, (ros::WallTime::now()-tm).toSec());
        _pointCloudPublisher.publish(_pointcloudout);
    }

    /// prune all the points that are inside the convex hulls of the robot links
    /// Uses a different timestamp for every laser point cloud
    void PruneWithAccurateTiming(const sensor_msgs::PointCloud& pointcloudin, vector<LASERPOINT>& vlaserpoints)
    {
        int istampchan = 0;
        while(istampchan < (int)pointcloudin.channels.size()) {
            if( pointcloudin.channels[istampchan].name == "stamps" )
                break;
            ++istampchan;
        }

        if( istampchan >= (int)pointcloudin.channels.size()) {
            RAVELOG_DEBUG("accurate timing needs 'stamp' channel to be published in point cloud, reverting to simple timing\n");
            PruneWithSimpleTiming(pointcloudin, vlaserpoints);
            return;
        }

        // look for timestamp channel
        float fdeltatime = 0;
        for(int i = 0; i < (int)pointcloudin.channels[istampchan].values.size(); ++i)
            fdeltatime = pointcloudin.channels[istampchan].values[i];
            
        if( fdeltatime == 0 ) {
            PruneWithSimpleTiming(pointcloudin, vlaserpoints);
            return;
        }

        vlaserpoints.resize(pointcloudin.points.size());
        float ideltatime=1.0f/fdeltatime;
        int index = 0;
        FOREACH(itp, pointcloudin.points) {
            vlaserpoints[index] = LASERPOINT(Vector(itp->x,itp->y,itp->z,1),pointcloudin.channels[istampchan].values[index]*ideltatime);
            ++index;
        }

        try {
            geometry_msgs::PoseStamped poseout, posestart,poseend;
            posestart.pose.orientation.w = 1;
            posestart.header = pointcloudin.header;
            poseend.pose.orientation.w = 1;
            poseend.header = pointcloudin.header;
            poseend.header.stamp += ros::Duration().fromSec(fdeltatime);
            FOREACH(ithull, _vLinkHulls) {
                _tf.transformPose(ithull->tfframe, posestart, poseout);
                ithull->tstart = GetTransform(poseout.pose);
                _tf.transformPose(ithull->tfframe, poseend, poseout);
                ithull->tend = GetTransform(poseout.pose);
            }
        }
        catch(tf::TransformException& ex) {
            RAVELOG_WARN("failed to get tf frame: %s\n", ex.what());
            return;
        }

        // points are independent from each and loop can be parallelized
        #pragma omp parallel for schedule(dynamic,64)
        for(int i = 0; i < (int)vlaserpoints.size(); ++i) {
            LASERPOINT& laserpoint = vlaserpoints[i];
            FOREACH(ithull, _vLinkHulls) {
                Transform tinv, tinvstart = ithull->tstart.inverse(), tinvend = ithull->tend.inverse();
                tinv.rot = quatSlerp(tinvstart.rot,tinvend.rot,laserpoint.time);
                tinv.trans = tinvstart.trans*(1-laserpoint.time) + tinvend.trans*laserpoint.time;

                bool bInside = true;
                FOREACH(itplane, ithull->vconvexhull) {
                    Vector v = tinv * laserpoint.pt; v.w = 1;
                    if( dot4(*itplane,v) > 0 ) {
                        bInside = false;
                        break;
                    }
                }

                if( bInside ) {
                    laserpoint.inside = 1;
                    break;
                }
            }
        }
    }

    /// prune all the points that are inside the convex hulls of the robot links
    /// Uses the header timestamp for all laser point clouds
    void PruneWithSimpleTiming(const sensor_msgs::PointCloud& pointcloudin, vector<LASERPOINT>& vlaserpoints)
    {
        tf::StampedTransform bttransform;
        vlaserpoints.resize(0);
        
        // compute new hulls
        try {
            geometry_msgs::PoseStamped poseout, posestart,poseend;
            posestart.pose.orientation.w = 1;
            posestart.header = pointcloudin.header;
            FOREACH(ithull, _vLinkHulls) {
                _tf.transformPose(ithull->tfframe, posestart, poseout);
                ithull->tstart = GetTransform(poseout.pose);
            }
//            stringstream ss;
//            ss << _vLinkHulls.front().tfframe << ": " << _vLinkHulls.front().tstart << endl;
//            RAVELOG_INFO(ss.str());
        }
        catch(tf::TransformException& ex) {
            RAVELOG_WARN("failed to get tf frame: %s\n", ex.what());
            return;
        }
        
        vlaserpoints.resize(pointcloudin.points.size());

        int index = 0;
        FOREACH(itp, pointcloudin.points)
            vlaserpoints[index++] = LASERPOINT(Vector(itp->x,itp->y,itp->z,1),0);

        FOREACH(ithull, _vLinkHulls) {
            TransformMatrix tinvstart = ithull->tstart;//.inverse();
            ithull->vnewconvexhull.resize(ithull->vconvexhull.size());
            vector<Vector>::iterator itnewplane = ithull->vnewconvexhull.begin();
            FOREACH(itplane, ithull->vconvexhull) {
                itnewplane->x = itplane->x*tinvstart.m[0]+itplane->y*tinvstart.m[4]+itplane->z*tinvstart.m[8];
                itnewplane->y = itplane->x*tinvstart.m[1]+itplane->y*tinvstart.m[5]+itplane->z*tinvstart.m[9];
                itnewplane->z = itplane->x*tinvstart.m[2]+itplane->y*tinvstart.m[6]+itplane->z*tinvstart.m[10];
                itnewplane->w = itplane->x*tinvstart.trans.x+itplane->y*tinvstart.trans.y+itplane->z*tinvstart.trans.z+itplane->w;
                ++itnewplane;
            }
        }
                    
        // points are independent from each and loop can be parallelized
        #pragma omp parallel for schedule(dynamic,64)
        for(int i = 0; i < (int)vlaserpoints.size(); ++i) {
            LASERPOINT& laserpoint = vlaserpoints[i];
            FOREACH(ithull, _vLinkHulls) {

                bool bInside = true;
                FOREACH(itplane, ithull->vnewconvexhull) {
                    if( dot4(*itplane,laserpoint.pt) > 0 ) {
                        bInside = false;
                        break;
                    }
                }

                if( bInside ) {
                    laserpoint.inside = 1;
                    break;
                }
            }
        }
    }

    bool compute_convex_hull(const vector<Vector>& verts, vector<Vector>& vconvexplanes)
    {
        if( verts.size() <= 3 )
            return false;

        vconvexplanes.resize(0);

        int dim = 3;  	              // dimension of points
        vector<coordT> qpoints(3*verts.size());
        for(size_t i = 0; i < verts.size(); ++i) {
            qpoints[3*i+0] = verts[i].x;
            qpoints[3*i+1] = verts[i].y;
            qpoints[3*i+2] = verts[i].z;
        }
        
        bool bSuccess = false;
        boolT ismalloc = 0;           // True if qhull should free points in qh_freeqhull() or reallocation  
        char flags[]= "qhull Tv"; // option flags for qhull, see qh_opt.htm 
        FILE *outfile = NULL;    // stdout, output from qh_produce_output(), use NULL to skip qh_produce_output()  
        FILE *errfile = tmpfile();    // stderr, error messages from qhull code  
        
        int exitcode= qh_new_qhull (dim, qpoints.size()/3, &qpoints[0], ismalloc, flags, outfile, errfile);
        if (!exitcode) { // no error
            vconvexplanes.reserve(100);

            facetT *facet;	          // set by FORALLfacets 
            FORALLfacets { // 'qh facet_list' contains the convex hull
                vconvexplanes.push_back(Vector(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset-_convexpadding));
            }

            bSuccess = true;
        }

        qh_freeqhull(!qh_ALL);
        int curlong, totlong;	  // memory remaining after qh_memfreeshort 
        qh_memfreeshort (&curlong, &totlong);
        if (curlong || totlong)
            RAVELOG_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
     
        fclose(errfile);
        return bSuccess;
    }

    static Transform GetTransform(const btTransform& bt)
    {
        btQuaternion q = bt.getRotation();
        btVector3 o = bt.getOrigin();
        return Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(o.x(),o.y(),o.z()));
    }
    static Transform GetTransform(const geometry_msgs::Pose& pose)
    {
        return Transform(Vector(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z), Vector(pose.position.x, pose.position.y, pose.position.z));
    }
};

int main(int argc, char ** argv)
{
    // parse the command line options
    string robotname;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("robotlinks_filter_node [--robotfile openravefile]\n"
                   "  Start a node to prune points that are on the robot surface.\n"
                   "  Currently the robot file specified has to be in OpenRAVE format\n"
                   "  (see openrave_robot_descirption ros package for details)\n");

            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
            robotname = argv[i+1];
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv,"robotlinks_filter");
    if( !ros::master::check() ) {
        return 1;
    }
    RaveInitialize(true);
    boost::shared_ptr<RobotLinksFilter> plinksfilter(new RobotLinksFilter(robotname));
    ros::spin();
    plinksfilter.reset();
    
    return 0;
}

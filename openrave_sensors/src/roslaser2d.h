// Copyright (c) 2009-2010 Rosen Diankov
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// \author Rosen Diankov
#ifndef OPENRAVE_ROSBASELASER_H
#define OPENRAVE_ROSBASELASER_H

#include "plugindefs.h"

#include <sensor_msgs/LaserScan.h>

/// Laser rotates around the zaxis and it's 0 angle is pointed toward the xaxis.
class ROSLaser2D : public SensorBase
{
 protected:
    class BaseLaser2DXMLReader : public BaseXMLReader
    {
    public:
    BaseLaser2DXMLReader(boost::shared_ptr<ROSLaser2D> psensor) : _psensor(psensor) {}
        virtual ~BaseLaser2DXMLReader() {}

        virtual ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support )
                    return PE_Support;
                return PE_Ignore;
            }

            ss.str("");
            return (name != "sensor" && name != "scantopic" && name != "show" && name != "color") ? PE_Support : PE_Pass;
        }

        virtual bool endElement(const std::string& name)
        {    
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(name) )
                    _pcurreader.reset();
                return false;
            }
            else if( name == "sensor" )
                return true;
            else if( name == "scantopic" )
                ss >> _psensor->scantopic;
            else if( name == "show" )
                ss >> _psensor->_bRender;
            else if( name == "color" ) {
                ss >> _psensor->_vColor.x >> _psensor->_vColor.y >> _psensor->_vColor.z;
                // ok if not everything specified
                if( !ss )
                    ss.clear();
            }
            else
                RAVELOG_WARNA(str(boost::format("bad tag: %s")%name));

            if( !ss )
                RAVELOG_WARNA(str(boost::format("error parsing %s\n")%name));

            return false;
        }

        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader )
                _pcurreader->characters(ch);
            else {
                ss.clear();
                ss << ch;
            }
        }

    protected:
        boost::shared_ptr<ROSLaser2D> _psensor;
        BaseXMLReaderPtr _pcurreader;
        stringstream ss;
    };

public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const std::list<std::pair<std::string,std::string> >& atts)
    {
        return BaseXMLReaderPtr(new BaseLaser2DXMLReader(boost::dynamic_pointer_cast<ROSLaser2D>(ptr)));
    }

    ROSLaser2D(EnvironmentBasePtr penv) : SensorBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\nConnects to a ROS LaserScan message that receives laser data in real-time and provides that information to the OpenRAVE environment";
        _pgeom.reset(new LaserGeomData());
        _pdata.reset(new LaserSensorData());

        _bDestroyThread = true;
        _bRender = false;
        _bPower = true;
        _bUpdatePlot = true;
        scantopic = "/scan";
        _vColor = RaveVector<float>(0.5f,0.5f,1,1);
        _pgeom->min_angle[0] = _pgeom->min_angle[1] = 0;
        _pgeom->max_angle[0] = _pgeom->max_angle[1] = 0;
        _pgeom->resolution[0] = _pgeom->resolution[1] = 0;
        _pgeom->max_range = 0;
    }
    virtual ~ROSLaser2D()
    {
        Reset(0);
    }
    
    virtual bool Init(const string& args)
    {
        Reset(1);
        return true;
    }

    virtual void Reset(int options)
    {
        boost::mutex::scoped_lock lock(_mutexdata);

        _listGraphicsHandles.clear();
        _iconhandle.reset();

        _bUpdatePlot = true;
        _pdata->positions.clear();
        _pdata->ranges.clear();
        _pdata->intensity.clear();

        _bDestroyThread = true;
        _submstate.shutdown();
        _node.reset();
        _threadsensor.join();
        
        if( options & 1 ) {
            startsubscriptions();
        }
    }

    virtual void startsubscriptions()
    {
        int argc=0;
        ros::init(argc,NULL,"openrave", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

        if( ros::master::check() ) {
            _node.reset(new ros::NodeHandle());
            _submstate = _node->subscribe(scantopic, 10, &ROSLaser2D::newscan_cb, this);

            _bDestroyThread = false;
            _threadsensor = boost::thread(boost::bind(&ROSLaser2D::_SensorThread, this));
        }
        else
            RAVELOG_WARNA("failed to detect ROS master");
    }

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        if( _bUpdatePlot ) {
            Transform t = GetLaserPlaneTransform();
    
            if( _pgeom->max_angle[0] > _pgeom->min_angle[0] ) {
                int N = 10;
                viconpoints.resize(N+2);
                viconindices.resize(3*N);
                viconpoints[0] = t.trans;
                Transform trot;

                for(int i = 0; i <= N; ++i) {
                    dReal fang = _pgeom->min_angle[0] + (_pgeom->max_angle[0]-_pgeom->min_angle[0])*(float)i/(float)N;
                    trot.rotfromaxisangle(Vector(0,0,1), fang);
                    viconpoints[i+1] = t * trot.rotate(Vector(0.05f,0,0));

                    if( i < N ) {
                        viconindices[3*i+0] = 0;
                        viconindices[3*i+1] = i+1;
                        viconindices[3*i+2] = i+2;
                    }
                }

                RaveVector<float> vcolor = _vColor*0.5f;
                vcolor.w = 0.7f;
                _iconhandle = GetEnv()->drawtrimesh(viconpoints[0], sizeof(viconpoints[0]), &viconindices[0], N, vcolor);
            }
        }

        return true;
    }


    virtual SensorGeometryPtr GetSensorGeometry()
    {
        LaserGeomData* pgeom = new LaserGeomData();
        *pgeom = *_pgeom;
        return SensorGeometryPtr(pgeom);
    }

    virtual SensorDataPtr CreateSensorData()
    {
        return SensorDataPtr(new LaserSensorData());
    }

    virtual bool GetSensorData(SensorDataPtr psensordata)
    {
        boost::mutex::scoped_lock lock(_mutexdata);
        *boost::dynamic_pointer_cast<LaserSensorData>(psensordata) = *_pdata;
        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        if( !is )
            throw openrave_exception("no command",ORE_InvalidArguments);
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "show" || cmd == "render" ) {
            is >> _bRender;
        }
        else if( cmd == "power") {
            is >> _bPower;
            if( _bPower && !_node ) {
                startsubscriptions();
            }
            else if( !_bPower && !!_node ) {
                _submstate.shutdown();
                _node.reset();
                boost::mutex::scoped_lock lock(_mutexdata);
                _listGraphicsHandles.resize(0);
            }
        }

        return !!is;
    }

    virtual void SetTransform(const Transform& trans)
    {
        _trans = trans;
        _bUpdatePlot = true;
    }

    virtual Transform GetTransform() { return _trans; }

 protected:

    virtual Transform GetLaserPlaneTransform() { return _trans; }
    virtual void newscan_cb(const sensor_msgs::LaserScanConstPtr& msg)
    {
        boost::mutex::scoped_lock lock(_mutexdata);
        _pgeom->min_angle[0] = msg->angle_min;
        _pgeom->max_angle[0] = msg->angle_max;
        _pgeom->resolution[0] = msg->angle_increment;
        _pgeom->max_range = msg->range_max;

        Transform t = _trans;
        _pdata->t = t;
        _pdata->positions.resize(msg->ranges.size());
        _pdata->ranges.resize(msg->ranges.size());
        _pdata->intensity.resize(msg->intensities.size());
    
        float ang = msg->angle_min;
        for(size_t i = 0; i < msg->ranges.size(); ++i, ang += msg->angle_increment) {
            _pdata->positions[i] = _trans.trans;
            float range = max(msg->range_min,min(msg->range_max,msg->ranges[i]));
            Vector v(range*cosf(ang),range*sinf(ang),0);
            _pdata->ranges[i] = t.rotate(v);
        }

        for(int i = 0; i < (int)_pdata->intensity.size(); ++i)
            _pdata->intensity[i] = msg->intensities[i];

        if( _bRender && msg->ranges.size() > 0 ) {

            // If can render, check if some time passed before last update
            list<EnvironmentBase::GraphHandlePtr> listhandles;
            int N = 0;
            vector<RaveVector<float> > vpoints;
            vector<int> vindices;

            {
                // Lock the data mutex and fill the arrays used for rendering
                N = (int)_pdata->ranges.size();
                vpoints.resize(N+1);
                ang = msg->angle_min;
                for(int i = 0; i < N; ++i, ang += msg->angle_increment) {
                    float range = max(msg->range_min,min(msg->range_max,msg->ranges[i]));
                    vpoints[i] = t * Vector(range*cosf(ang),range*sinf(ang),0);
                }
                vpoints[N] = t.trans;
            }

            // render the transparent fan
            vindices.resize(3*(N-1));
            
            for(int i = 0; i < N-1; ++i) {
                vindices[3*i+0] = i;
                vindices[3*i+1] = i+1;
                vindices[3*i+2] = N;
            }

            _vColor.w = 1;
            // Render points at each measurement, and a triangle fan for the entire free surface of the laser
            listhandles.push_back(GetEnv()->plot3(&vpoints[0].x, N, sizeof(vpoints[0]), 5.0f, _vColor));
            
            _vColor.w = 0.2f;
            listhandles.push_back(GetEnv()->drawtrimesh(vpoints[0], sizeof(vpoints[0]), &vindices[0], N-1, _vColor));
            
            _listGraphicsHandles.swap(listhandles);
        }
        else { // destroy graphs
            _listGraphicsHandles.clear();
        }
    }

    void _SensorThread()
    {
        while(!_bDestroyThread) {
            ros::spinOnce();
            usleep(1000); // query every 1ms
        }
    }

    boost::shared_ptr<ros::NodeHandle> _node;
    ros::Subscriber _submstate;

    boost::shared_ptr<LaserGeomData> _pgeom;
    boost::shared_ptr<LaserSensorData> _pdata;

    // more geom stuff
    string scantopic;
    RaveVector<float> _vColor;
    
    Transform _trans;
    list<EnvironmentBase::GraphHandlePtr> _listGraphicsHandles;
    EnvironmentBase::GraphHandlePtr _iconhandle;
    vector<RaveVector<float> > viconpoints;
    vector<int> viconindices;

    boost::mutex _mutexdata;
    boost::thread _threadsensor;
    bool _bDestroyThread;

    bool _bRender, _bPower;
    bool _bUpdatePlot;

    friend class BaseLaser2DXMLReader;
};

#endif

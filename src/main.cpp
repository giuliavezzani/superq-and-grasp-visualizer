/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/clustering.h>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkSuperquadric.h>
#include <vtkUnsignedCharArray.h>
#include <vtkTransform.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkMatrix4x4.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkPlaneSource.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

Mutex mutex;

/****************************************************************/
class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    /****************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /****************************************************************/
    static UpdateCommand *New()
    {
        return new UpdateCommand;
    }

    /****************************************************************/
    UpdateCommand() : closing(nullptr) { }

    /****************************************************************/
    void set_closing(const bool &closing)
    {
        this->closing=&closing;
    }

    /****************************************************************/
    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId), 
                 void *vtkNotUsed(callData))
    {
        LockGuard lg(mutex);
        vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);
        if (closing!=nullptr)
        {
            if (*closing)
            {
                iren->GetRenderWindow()->Finalize();
                iren->TerminateApp();
                return;
            }
        }

        iren->GetRenderWindow()->SetWindowName("Superquadric visualizer");
        iren->Render();
    }
};

/****************************************************************/
class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    /****************************************************************/
    vtkSmartPointer<vtkActor> &get_actor()
    {
        return vtk_actor;
    }
};

/****************************************************************/
class Points : public Object
{
protected:
    vtkSmartPointer<vtkPoints> vtk_points;
    vtkSmartPointer<vtkUnsignedCharArray> vtk_colors;
    vtkSmartPointer<vtkPolyData> vtk_polydata;
    vtkSmartPointer<vtkVertexGlyphFilter> vtk_glyphFilter;

public:
    /****************************************************************/
    Points(const vector<Vector> &points, const int point_size)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }

    /****************************************************************/
    void set_points(const vector<Vector> &points)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (size_t i=0; i<points.size(); i++)
            vtk_points->InsertNextPoint(points[i][0],points[i][1],points[i][2]);

        vtk_polydata->SetPoints(vtk_points);
    }

    /****************************************************************/
    bool set_colors(const vector<vector<unsigned char>> &colors)
    {
        if (colors.size()==vtk_points->GetNumberOfPoints())
        {
            vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
            vtk_colors->SetNumberOfComponents(3);
            for (size_t i=0; i<colors.size(); i++)
                vtk_colors->InsertNextTypedTuple(colors[i].data());

            vtk_polydata->GetPointData()->SetScalars(vtk_colors);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    vtkSmartPointer<vtkPolyData> &get_polydata()
    {
        return vtk_polydata;
    }
};

/****************************************************************/
class Plane : public Object
{
protected:
    vtkSmartPointer<vtkPlaneSource> plane_source;
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;

public:
    /****************************************************************/
    Plane(double z_height)
    {
        plane_source = vtkSmartPointer<vtkPlaneSource>::New();
        plane_source->SetCenter(0.0, 0.0, z_height);
        plane_source->SetNormal(0.0, 0.0, 1.0);
        plane_source->Update();

        vtkPolyData* plane = plane_source->GetOutput();

        // Create a mapper and actor
        vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputData(plane);

        vtk_actor = vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
    }

};


/****************************************************************/
class Superquadric : public Object
{
protected:
    vtkSmartPointer<vtkSuperquadric> vtk_superquadric;
    vtkSmartPointer<vtkSampleFunction> vtk_sample;
    vtkSmartPointer<vtkContourFilter> vtk_contours;
    vtkSmartPointer<vtkTransform> vtk_transform;

public:
    /****************************************************************/
    Superquadric(const Vector &r)
    {
        double bx=2.0*r[7];
        double by=2.0*r[8];
        double bz=2.0*r[9];

        vtk_superquadric=vtkSmartPointer<vtkSuperquadric>::New();
        vtk_superquadric->ToroidalOff();
        vtk_superquadric->SetSize(1.0);
        vtk_superquadric->SetCenter(zeros(3).data());

        vtk_superquadric->SetScale(r[7],r[8],r[9]);
        vtk_superquadric->SetPhiRoundness(r[10]);
        vtk_superquadric->SetThetaRoundness(r[11]);

        vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
        vtk_sample->SetSampleDimensions(50,50,50);
        vtk_sample->SetImplicitFunction(vtk_superquadric);
        vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

        // The isosurface is defined at 0.0 as specified in
        // https://github.com/Kitware/VTK/blob/master/Common/DataModel/vtkSuperquadric.cxx
        vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
        vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
        vtk_contours->GenerateValues(1,0.0,0.0);

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
        vtk_mapper->ScalarVisibilityOff();

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetColor(0.0,0.3,0.6);
        vtk_actor->GetProperty()->SetOpacity(0.25);

        vtk_transform=vtkSmartPointer<vtkTransform>::New();
        vtk_transform->Translate(r.subVector(0,2).data());       
        vtk_transform->RotateWXYZ((180.0/M_PI)*r[6],r.subVector(3,5).data());
        vtk_actor->SetUserTransform(vtk_transform);
    }

    /****************************************************************/
    void set_parameters(const Vector &r)
    {
        double bx=2.0*r[7];
        double by=2.0*r[8];
        double bz=2.0*r[9];

        vtk_superquadric->SetScale(r[7],r[8],r[9]);
        vtk_superquadric->SetPhiRoundness(r[10]);
        vtk_superquadric->SetThetaRoundness(r[11]);
        
        vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

        vtk_transform->Identity();
        vtk_transform->Translate(r.subVector(0,2).data());
        vtk_transform->RotateWXYZ((180.0/M_PI)*r[6],r.subVector(3,5).data());
    }
};

/****************************************************************/
class GraspPose : public Object
{
public:
    //  essential parameters for representing a grasping pose
    vtkSmartPointer<vtkAxesActor> pose_vtk_actor;
    vtkSmartPointer<vtkCaptionActor2D> pose_vtk_caption_actor;
    vtkSmartPointer<vtkTransform> pose_vtk_transform;
    yarp::sig::Matrix pose;


    /****************************************************************/
    GraspPose() :  pose(4,4)
    {
        pose.eye();
        pose_vtk_actor = vtkSmartPointer<vtkAxesActor>::New();
        pose_vtk_transform = vtkSmartPointer<vtkTransform>::New();
        pose_vtk_caption_actor = vtkSmartPointer<vtkCaptionActor2D>::New();
    }

    /****************************************************************/
    void setvtkTransform(const Vector &pose_vect)
    {
        pose=euler2dcm(pose_vect.subVector(3,5));
        pose.setSubcol(pose_vect.subVector(0,2), 0,3);

        vtkSmartPointer<vtkMatrix4x4> m_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
        m_vtk->Zero();
        for (size_t i = 0; i < 4; i++)
        {
            for(size_t j = 0; j < 4; j++)
            {
                m_vtk->SetElement(i, j, pose(i, j));
            }
        }

        pose_vtk_transform->SetMatrix(m_vtk);
    }

    /****************************************************************/
    void setvtkActorCaption(const string &caption)
    {
        pose_vtk_caption_actor->GetTextActor()->SetTextScaleModeToNone();
        pose_vtk_caption_actor->SetCaption(caption.c_str());
        pose_vtk_caption_actor->BorderOff();
        pose_vtk_caption_actor->LeaderOn();
        pose_vtk_caption_actor->GetCaptionTextProperty()->SetFontSize(20);
        pose_vtk_caption_actor->GetCaptionTextProperty()->FrameOff();
        pose_vtk_caption_actor->GetCaptionTextProperty()->ShadowOff();
        pose_vtk_caption_actor->GetCaptionTextProperty()->BoldOff();
        pose_vtk_caption_actor->GetCaptionTextProperty()->ItalicOff();
        pose_vtk_caption_actor->SetAttachmentPoint(pose(0,3), pose(1,3), pose(2,3));
    }

};


/****************************************************************/
class Visualizer : public RFModule
{
    Bottle outliersRemovalOptions;
    unsigned int uniform_sample;
    double random_sample;
    bool from_file;
    bool get_grasping_pose;
    bool visualize_hand;
    bool viewer_enabled;
    bool closing;
    string hand_for_computation;
    int num_superq;
    std::deque<std::string> names;
    vector<Vector> poses, hands;
    vector<double> costs;
    vector<double> dims;

    /*class PointsProcessor : public PortReader {
        Visualizer *visualizer;
        bool read(ConnectionReader &connection) override {
            PointCloud<DataXYZRGBA> points;
            if (!points.read(connection))
                return false;
            Bottle reply;
            visualizer->process(points,reply);
            if (ConnectionWriter *writer=connection.getWriter())
                reply.write(*writer);
            return true;
        }
    public:
        PointsProcessor(Visualizer *visualizer_) : visualizer(visualizer_) { }
    } pointsProcessor;*/

    RpcServer rpcPoints,rpcService;

    vector<Vector> all_points,in_points,out_points,dwn_points, points_hand;
    vector<vector<unsigned char>> all_colors;
    
    unique_ptr<Points> vtk_all_points,vtk_out_points,vtk_dwn_points, vtk_points_hand;
    unique_ptr<Superquadric> vtk_superquadric;
    unique_ptr<Plane> vtk_plane;

    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

    //BufferedPort<Bottle> oPort;
    //BufferedPort<Property> iPort;

    RpcClient superqRpc;
    RpcClient graspRpc;

    // For looking at the object to be modeled
    RpcClient action_render_rpc;

    // For getting the point cloud
    RpcClient point_cloud_rpc;

    /****************************************************************/
    void removeOutliers()
    {
        double t0=Time::now();
        if (outliersRemovalOptions.size()>=2)
        {
            double radius=outliersRemovalOptions.get(0).asDouble();
            int minpts=outliersRemovalOptions.get(1).asInt();

            Property options;
            options.put("epsilon",radius);
            options.put("minpts",minpts);

            DBSCAN dbscan;
            map<size_t,set<size_t>> clusters=dbscan.cluster(all_points,options);

            size_t largest_class; size_t largest_size=0;
            for (auto it=begin(clusters); it!=end(clusters); it++)
            {
                if (it->second.size()>largest_size)
                {
                    largest_size=it->second.size();
                    largest_class=it->first;
                }
            }

            auto &c=clusters[largest_class];
            for (size_t i=0; i<all_points.size(); i++)
            {
                if (c.find(i)==end(c))
                    out_points.push_back(all_points[i]);
                else
                    in_points.push_back(all_points[i]);
            }
        }
        else
            in_points=all_points;

        double t1=Time::now();
        yInfo()<<out_points.size()<<"outliers removed out of"
               <<all_points.size()<<"points in"<<t1-t0<<"[s]";
    }

    /****************************************************************/
    void sampleInliers()
    {
        double t0=Time::now();
        if (random_sample>=1.0)
        {
            unsigned int cnt=0;
            for (auto &p:in_points)
            {
                if ((cnt++%uniform_sample)==0)
                    dwn_points.push_back(p);
            }
        }
        else
        {
            set<unsigned int> idx;
            while (idx.size()<(size_t)(random_sample*in_points.size()))
            {
                unsigned int i=(unsigned int)(Rand::scalar(0.0,1.0)*in_points.size());
                if (idx.find(i)==idx.end())
                {
                    dwn_points.push_back(in_points[i]);
                    idx.insert(i);
                }
            }
        }

        double t1=Time::now();
        yInfo()<<dwn_points.size()<<"samples out of"
               <<in_points.size()<<"inliers in"<<t1-t0<<"[s]";
    }

    /****************************************************************/
    vector<Vector>  getBottle(Bottle &estimated_superq)
    {
        Vector superq_aux(12,0.0);
        vector<Vector> superqs;

        Bottle *all=estimated_superq.get(0).asList();

        if (all->size() == 4)
        {
            for (size_t i=0; i<all->size(); i++)
            {
                Bottle *group=all->get(i).asList();
                if (group->get(0).asString() == "dimensions")
                {
                    Bottle *dim=group->get(1).asList();

                    superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
                }
                else if (group->get(0).asString() == "exponents")
                {
                    Bottle *dim=group->get(1).asList();

                    superq_aux[3]=dim->get(0).asDouble(); superq_aux[4]=dim->get(1).asDouble();
                }
                else if (group->get(0).asString() == "center")
                {
                    Bottle *dim=group->get(1).asList();

                    superq_aux[5]=dim->get(0).asDouble(); superq_aux[6]=dim->get(1).asDouble(); superq_aux[7]=dim->get(2).asDouble();
                }
                else if (group->get(0).asString() == "orientation")
                {
                    Bottle *dim=group->get(1).asList();

                    superq_aux[8]=dim->get(0).asDouble(); superq_aux[9]=dim->get(1).asDouble(); superq_aux[10]=dim->get(2).asDouble(); superq_aux[11]=dim->get(3).asDouble();
                }
            }
            superqs.push_back(superq_aux);
        }
        else
        {
            int size=all->size()/4;

            for (size_t i=1; i<=size; i++)
            {
                stringstream ss;
                ss<<i;
                string i_str=ss.str();

                for (size_t i=0; i<all->size(); i++)
                {
                    Bottle *group=all->get(i).asList();
                    if (group->get(0).asString() == "dimensions_"+i_str)
                    {
                        Bottle *dim=group->get(1).asList();

                        superq_aux[0]=dim->get(0).asDouble(); superq_aux[1]=dim->get(1).asDouble(); superq_aux[2]=dim->get(2).asDouble();
                    }
                    if (group->get(0).asString() == "exponent_"+i_str)
                    {
                        Bottle *dim=group->get(1).asList();

                        superq_aux[3]=dim->get(0).asDouble(); superq_aux[4]=dim->get(1).asDouble();
                    }
                    if (group->get(0).asString() == "center_"+i_str)
                    {
                        Bottle *dim=group->get(1).asList();

                        superq_aux[5]=dim->get(0).asDouble(); superq_aux[6]=dim->get(1).asDouble(); superq_aux[7]=dim->get(2).asDouble();
                    }
                    if (group->get(0).asString() == "orientation_"+i_str)
                    {
                        Bottle *dim=group->get(1).asList();

                        superq_aux[8]=dim->get(0).asDouble(); superq_aux[9]=dim->get(1).asDouble(); superq_aux[10]=dim->get(2).asDouble(); superq_aux[11]=dim->get(3).asDouble();
                    }
                }

                superqs.push_back(superq_aux);
            }
        }

        return superqs;
    }

    /****************************************************************/
    void getPose(Bottle &pose_bottle, const string &tag, const string &hand, vector<Vector> &poses)
    {
        Bottle *all=pose_bottle.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            Vector pose(6,0.0);

            Bottle *group=all->get(i).asList(); 

            for (size_t l=0; l<num_superq; l++)
            {
                stringstream ss;
                ss<<l;
                if (group->get(0).asString() == tag+"_"+ss.str()+"_"+hand)
                {
                    Bottle *dim=group->get(1).asList();                

                    for (size_t l=0; l<dim->size(); l++)
                    {
                        pose[l]=dim->get(l).asDouble();
                    }
                }
            }

            if (norm(pose) > 0.0)
            {
                poses.push_back(pose);

                if (tag=="pose")
                    names.push_back(hand);
            }
        }

    }

    /****************************************************************/
    void  getCost(Bottle &cost_bottle, const string &tag, const string &hand, vector<double> &costs)
    {
        Bottle *all=cost_bottle.get(0).asList();

        for (size_t i=0; i<all->size(); i++)
        {
            double c=-1.0;

            Bottle *group=all->get(i).asList();

            for (size_t l=0; l<num_superq; l++)
            {
                stringstream ss;
                ss<<l;

                if (group->get(0).asString() == tag+"_"+ss.str()+"_"+hand)
                {
                    c=group->get(1).asDouble();
                }
            }
            if (c > 0.0)
            {
                costs.push_back(c);
            }
        }
    }

    /****************************************************************/
    void askOnePose(vector<Vector> &object, Bottle &cmd)
    {
        cmd.clear();
        cmd.addString("get_grasping_pose");

        Vector sup(12,0.0);
        sup=object[0];
        Bottle &b1=cmd.addList();
        Bottle &b2=b1.addList();
        b2.addString("dimensions");
        Bottle &b2l=b2.addList();
        b2l.addDouble(sup[0]); b2l.addDouble(sup[1]); b2l.addDouble(sup[2]);

        Bottle &b3=b1.addList();
        b3.addString("exponents");
        Bottle &b3l=b3.addList();
        b3l.addDouble(sup[3]); b3l.addDouble(sup[4]);

        Bottle &b4=b1.addList();
        b4.addString("center");
        Bottle &b4l=b4.addList();
        b4l.addDouble(sup[5]); b4l.addDouble(sup[6]); b4l.addDouble(sup[7]);

        Bottle &b5=b1.addList();
        b5.addString("orientation");
        Bottle &b5l=b5.addList();
        b5l.addDouble(sup[8]); b5l.addDouble(sup[9]); b5l.addDouble(sup[10]); b5l.addDouble(sup[11]);

        cmd.addString(hand_for_computation);
    }

    /****************************************************************/
    void askMultiplePose(vector<Vector> &object, Bottle &cmd)
    {
        cmd.clear();
        cmd.addString("get_grasping_pose_multiple");

        Bottle &b1=cmd.addList();
        Bottle &b2=b1.addList();
        b2.addString("dimensions");
        Bottle &b2l=b2.addList();
        b2l.addDouble(object[0][0]); b2l.addDouble(object[0][1]); b2l.addDouble(object[0][2]);

        Bottle &b3=b1.addList();
        b3.addString("exponents");
        Bottle &b3l=b3.addList();
        b3l.addDouble(object[0][3]); b3l.addDouble(object[0][4]);

        Bottle &b4=b1.addList();
        b4.addString("center");
        Bottle &b4l=b4.addList();
        b4l.addDouble(object[0][5]); b4l.addDouble(object[0][6]); b4l.addDouble(object[0][7]);

        Bottle &b5=b1.addList();
        b5.addString("orientation");
        Bottle &b5l=b5.addList();
        b5l.addDouble(object[0][8]); b5l.addDouble(object[0][9]); b5l.addDouble(object[0][10]); b5l.addDouble(object[0][11]);

        Bottle &bb1=cmd.addList();
        Bottle &bb2=bb1.addList();
        bb2.addString("obstacles");
        Bottle &bb3=bb2.addList();

        for (size_t j=1; j<object.size(); j++)
        {
            Bottle &bb4=bb3.addList();

            for (size_t k=0; k<12; k++)
            {
                bb4.addDouble(object[j][k]);
            }
        }

        cmd.addString(hand_for_computation);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        Rand::init();

        from_file=rf.check("file");
        get_grasping_pose=rf.check("get_grasping_pose");
        visualize_hand=rf.check("visualize_hand");

        if (from_file)
        {
            string file=rf.find("file").asString();
            ifstream fin(file.c_str());
            if (!fin.is_open())
            {
                yError()<<"Unable to open file \""<<file<<"\"";
                return false;
            }

            Vector p(3);
            vector<unsigned int> c_(3);
            vector<unsigned char> c(3);

            string line;
            while (getline(fin,line))
            {
                istringstream iss(line);
                if (!(iss>>p[0]>>p[1]>>p[2]))
                    break;
                all_points.push_back(p);

                fill(c_.begin(),c_.end(),120);
                iss>>c_[0]>>c_[1]>>c_[2];
                c[0]=(unsigned char)c_[0];
                c[1]=(unsigned char)c_[1];
                c[2]=(unsigned char)c_[2];
                all_colors.push_back(c);
            }
        }
        else
        {
            //rpcPoints.open("/superq-and-grasp-visualizer/points:rpc");
            //rpcPoints.setReader(pointsProcessor);

            rpcService.open("/superq-and-grasp-visualizer/service:rpc");
            attach(rpcService);

            action_render_rpc.open("/superq-and-grasp-visualizer/actionRenderer:rpc");
            point_cloud_rpc.open("/superq-and-grasp-visualizer/pointCloud:rpc");
        }

        //oPort.open("/superquadric-visualizer:o");
        //iPort.open("/superquadric-visualizer:i");


        /*if (rf.check("streaming_mode"))
        {

        if (!Network::connect(oPort.getName(),"/superquadric-model/point:i") ||
                !Network::connect("/superquadric-model/superq:o",iPort.getName()))
            {
                yError()<<"Unable to connect to superquadric-model";
                close();
                return false;
            }
        }
        else
        {*/
            superqRpc.open("/superq-and-grasp-visualizer/rpc:i");
            if (!Network::connect(superqRpc.getName(),"/superquadric-model/rpc"))
            {
                yError()<<"Unable to connect to superquadric-model rpc ";
                close();
                return false;
            }
        //}

        if (rf.check("get_grasping_pose"))
        {
            graspRpc.open("/test-grasp/rpc:i");
            if (!Network::connect(graspRpc.getName(),"/superquadric-grasp/rpc"))
            {
                yError()<<"Unable to connect to superquadric-grasp rpc ";
                close();
                return false;
            }

            hand_for_computation=rf.check("hand", Value("right")).asString();
        }


        if (rf.check("remove-outliers"))
            if (const Bottle *ptr=rf.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;

        uniform_sample=(unsigned int)rf.check("uniform-sample",Value(1)).asInt();
        random_sample=rf.check("random-sample",Value(1.0)).asDouble();
        viewer_enabled=!rf.check("disable-viewer");

        vector<double> backgroundColor={0.7,0.7,0.7};
        if (rf.check("background-color"))
        {
            if (const Bottle *ptr=rf.find("background-color").asList())
            {
                size_t len=std::min(backgroundColor.size(),ptr->size());
                for (size_t i=0; i<len; i++)
                    backgroundColor[i]=ptr->get(i).asDouble();
            }
        }

        removeOutliers();
        sampleInliers();


        vtk_all_points=unique_ptr<Points>(new Points(all_points,2));
        vtk_out_points=unique_ptr<Points>(new Points(out_points,4));
        vtk_dwn_points=unique_ptr<Points>(new Points(dwn_points,1));

        vtk_all_points->set_colors(all_colors);
        vtk_out_points->get_actor()->GetProperty()->SetColor(1.0,0.0,0.0);
        vtk_dwn_points->get_actor()->GetProperty()->SetColor(1.0,1.0,0.0);

        vtk_plane=unique_ptr<Plane>(new Plane(-0.18));

        /** Test **/
        vtk_renderer=vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow=vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindowInteractor=vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

        vtk_renderWindow->SetSize(600,600);
        vtk_renderWindow->AddRenderer(vtk_renderer);

        vtk_renderer->AddActor(vtk_all_points->get_actor());
        vtk_renderer->AddActor(vtk_out_points->get_actor());
        if (dwn_points.size()!=in_points.size())
            vtk_renderer->AddActor(vtk_dwn_points->get_actor());

        vtk_renderer->SetBackground(backgroundColor.data());

        vtk_axes=vtkSmartPointer<vtkAxesActor>::New();
        vtk_widget=vtkSmartPointer<vtkOrientationMarkerWidget>::New();
        vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
        vtk_widget->SetOrientationMarker(vtk_axes);
        vtk_widget->SetInteractor(vtk_renderWindowInteractor);
        vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
        vtk_widget->SetEnabled(1);
        vtk_widget->InteractiveOn();

        vtk_style=vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

        if (dwn_points.size()>0)
        {
            computeAndVisualizeSuperqAndGrasp();
        }

        if (viewer_enabled)
        {
            vtk_renderWindowInteractor->Initialize();
            vtk_renderWindowInteractor->CreateRepeatingTimer(10);

            vtk_updateCallback=vtkSmartPointer<UpdateCommand>::New();
            vtk_updateCallback->set_closing(closing);
            vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
            vtk_renderWindowInteractor->Start();

        }

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 1.0;
    }

    /****************************************************************/
    bool updateModule() override
    {
        return (!from_file && !viewer_enabled);
    }

    /****************************************************************/
   /* void process(const PointCloud<DataXYZRGBA> &points, Bottle &reply)
    {   
        reply.clear();
        if (points.size()>0)
        {
            LockGuard lg(mutex);

            all_points.clear();
            all_colors.clear();
            in_points.clear();
            out_points.clear();
            dwn_points.clear();

            Vector p(3);
            vector<unsigned char> c(3);
            for (int i=0; i<points.size(); i++)
            {
                p[0]=points(i).x;
                p[1]=points(i).y;
                p[2]=points(i).z;
                c[0]=points(i).r;
                c[1]=points(i).g;
                c[2]=points(i).b;
                all_points.push_back(p);
                all_colors.push_back(c);
            }

            removeOutliers();
            sampleInliers();
            
            vtk_all_points->set_points(all_points);
            vtk_all_points->set_colors(all_colors);
            vtk_out_points->set_points(out_points);
            vtk_dwn_points->set_points(dwn_points);

            Vector r;

            if (dwn_points.size()>0)
            {
                Bottle cmd, superq_b;
                cmd.addString("send_point_clouds");

                Bottle &in1=cmd.addList();

                for (size_t i=0; i<dwn_points.size(); i++)
                {
                    Bottle &in=in1.addList();
                    in.addDouble(dwn_points[i][0]);
                    in.addDouble(dwn_points[i][1]);
                    in.addDouble(dwn_points[i][2]);
                    in.addDouble(dwn_points[i][3]);
                    in.addDouble(dwn_points[i][4]);
                    in.addDouble(dwn_points[i][5]);
                }

                superqRpc.write(cmd, superq_b);

                cmd.clear();
                cmd.addString("get_superq");
                superqRpc.write(cmd, superq_b);

                vector<Vector> v;
                v = getBottle(superq_b);
                for (size_t i=0; i< v.size(); i++)
                {
                    r.resize(12,0.0);
                    //Vector orient = dcm2euler(axis2dcm(v.subVector(8,11)));

                    r.setSubvector(0, v[i].subVector(5,7));
                    r.setSubvector(3, v[i].subVector(8,11));
                    r.setSubvector(7, v[i].subVector(0,2));
                    r.setSubvector(10, v[i].subVector(3,4));

                    // TO FIX
                    yInfo()<<"Read superquadric: "<<r.toString();
                    vtk_superquadric->set_parameters(r);
                }

            }

            reply.read(r);
        }
    }*/

    /****************************************************************/
    bool computeAndVisualizeSuperqAndGrasp()
    {
        Bottle cmd, superq_b, reply;
        //cmd.addString("send_point_clouds");

        // Test
        cmd.addString("get_superq");

        Bottle &in1=cmd.addList();

        for (size_t i=0; i<dwn_points.size(); i++)
        {
            Bottle &in=in1.addList();
            in.addDouble(dwn_points[i][0]);
            in.addDouble(dwn_points[i][1]);
            in.addDouble(dwn_points[i][2]);
            in.addDouble(dwn_points[i][3]);
            in.addDouble(dwn_points[i][4]);
            in.addDouble(dwn_points[i][5]);
        }

        //superqRpc.write(cmd, superq_b);

        //cmd.clear();


        superqRpc.write(cmd, superq_b);

        Vector r;
        vector<Vector> v=getBottle(superq_b);

        num_superq=v.size();

        if (get_grasping_pose)
        {
            if (v.size()==1)
            {
                askOnePose(v, cmd);
            }
            else
            {
                askMultiplePose(v,cmd);
            }

            yInfo()<<"Command asked "<<cmd.toString();

            graspRpc.write(cmd, reply);

            yInfo()<<"Received solution: "<<reply.toString();

            names.clear();
            poses.clear();
            hands.clear();
            costs.clear();
            dims.clear();

            if (hand_for_computation!="both")
            {
                getPose(reply, "pose", hand_for_computation, poses);
                getPose(reply, "solution", hand_for_computation,  hands);
                getCost(reply, "cost", hand_for_computation, costs);
                getCost(reply, "hand_length", hand_for_computation, dims);
            }
            else
            {
                getPose(reply, "pose", "right",poses);
                getPose(reply, "solution", "right", hands);
                getCost(reply, "cost", "right", costs);
                getCost(reply, "hand_length", "right", dims);
                getPose(reply, "pose", "left", poses);
                getPose(reply, "solution", "left", hands);
                getCost(reply, "cost", "left", costs);
                getCost(reply, "hand_length", "left", dims);
            }
        }

        //  grasping pose candidates
        vector<shared_ptr<GraspPose>> pose_candidates;

        //  grasp pose actors (temporary fix)
        vector<vtkSmartPointer<vtkAxesActor>> pose_actors;
        vector<vtkSmartPointer<vtkCaptionActor2D>> pose_captions;

        for (size_t i=0; i< v.size(); i++)
        {
            r.resize(12,0.0);
            r.setSubvector(0, v[i].subVector(5,7));
            r.setSubvector(3, v[i].subVector(8,11));
            r.setSubvector(7, v[i].subVector(0,2));
            r.setSubvector(10, v[i].subVector(3,4));

            vtk_superquadric=unique_ptr<Superquadric>(new Superquadric(r));

            vtk_renderer->AddActor(vtk_superquadric->get_actor());

            vector<double> bounds(6),centroid(3);
            vtk_all_points->get_polydata()->GetBounds(bounds.data());
            for (size_t i=0; i<centroid.size(); i++)
                centroid[i]=0.5*(bounds[i<<1]+bounds[(i<<1)+1]);

            vtk_camera=vtkSmartPointer<vtkCamera>::New();
            vtk_camera->SetPosition(centroid[0]+1.0,centroid[1],centroid[2]+0.5);
            vtk_camera->SetFocalPoint(centroid.data());
            vtk_camera->SetViewUp(0.0,0.0,1.0);
            vtk_renderer->SetActiveCamera(vtk_camera);
        }

        if (poses.size()> 0)
            vtk_renderer->AddActor(vtk_plane->get_actor());


        for (size_t i=0; i< poses.size();i++)
        {
            points_hand.clear();

            yDebug()<<"Pose "<<i<< " "<<poses[i].toString();
            yDebug()<<"Cost "<<i<< " "<<costs[i];
            yDebug()<<"Name "<<i<< " "<<names[i];
            vtkSmartPointer<vtkAxesActor> ax_actor = vtkSmartPointer<vtkAxesActor>::New();
            vtkSmartPointer<vtkCaptionActor2D> cap_actor = vtkSmartPointer<vtkCaptionActor2D>::New();
            ax_actor->VisibilityOff();
            cap_actor->VisibilityOff();
            pose_actors.push_back(ax_actor);
            pose_captions.push_back(cap_actor);
            vtk_renderer->AddActor(pose_actors[i]);
            vtk_renderer->AddActor(pose_captions[i]);

            shared_ptr<GraspPose> candidate_pose = shared_ptr<GraspPose>(new GraspPose);

            candidate_pose->setvtkTransform(poses[i]);
            candidate_pose->pose_vtk_actor->SetUserTransform(candidate_pose->pose_vtk_transform);
            pose_actors[i]->SetUserTransform(candidate_pose->pose_vtk_transform);

            candidate_pose->pose_vtk_actor->ShallowCopy(pose_actors[i]);
            pose_actors[i]->AxisLabelsOff();
            pose_actors[i]->SetTotalLength(0.02, 0.02, 0.02);
            pose_actors[i]->VisibilityOn();

            pose_captions[i]->VisibilityOn();
            pose_captions[i]->GetTextActor()->SetTextScaleModeToNone();



            stringstream ss;
            if (hand_for_computation!="both")
                ss<<"pose_"<<i%num_superq<<"_"<<hand_for_computation<<"/ cost: "<<costs[i];
            else
            {
               ss<<"pose_"<<i%num_superq<<"_"+names[i]+" / cost: "<<setprecision(3)<<costs[i];
            }

            candidate_pose->setvtkActorCaption(ss.str());
            pose_captions[i]->SetCaption(candidate_pose->pose_vtk_caption_actor->GetCaption());
            pose_captions[i]->BorderOff();
            pose_captions[i]->LeaderOn();
            pose_captions[i]->GetCaptionTextProperty()->SetFontSize(15);
            pose_captions[i]->GetCaptionTextProperty()->FrameOff();
            pose_captions[i]->GetCaptionTextProperty()->ShadowOff();
            pose_captions[i]->GetCaptionTextProperty()->BoldOff();
            pose_captions[i]->GetCaptionTextProperty()->ItalicOff();
            pose_captions[i]->GetCaptionTextProperty()->SetColor(0.1, 0.1, 0.1);
            pose_captions[i]->SetAttachmentPoint(candidate_pose->pose_vtk_caption_actor->GetAttachmentPoint());

            if ("visualize_hand")
            {
                Vector pose_hand(12,0.0);
                pose_hand.setSubvector(0,hands[i].subVector(0,2));
                pose_hand.setSubvector(3, dcm2axis(euler2dcm(hands[i].subVector(3,5))));
                // Hand dimensions
                pose_hand[7]=0.03;
                pose_hand[8]=dims[i];
                pose_hand[9]=0.03;
                pose_hand[10]=pose_hand[11]=1.0;

                samplePointsHand(pose_hand, points_hand, names[i]);

                vtk_points_hand=unique_ptr<Points>(new Points(points_hand,8));

                vtk_renderer->AddActor(vtk_points_hand->get_actor());

                vtk_superquadric=unique_ptr<Superquadric>(new Superquadric(pose_hand));

                vtk_renderer->AddActor(vtk_superquadric->get_actor());
            }

            pose_candidates.push_back(candidate_pose);
        }
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply) override
    {
        LockGuard lg(mutex);

        string object;

        bool ok=false;
        if (command.check("remove-outliers"))
        {
            if (const Bottle *ptr=command.find("remove-outliers").asList())
                outliersRemovalOptions=*ptr;
            ok=true;
        }

        if (command.check("uniform-sample"))
        {
            uniform_sample=(unsigned int)command.find("uniform-sample").asInt();
            ok=true;
        }

        if (command.check("random-sample"))
        {
            random_sample=command.find("random-sample").asDouble();
            ok=true;
        }

        if (command.get(0).toString()=="compute-superq")
        {
            if (command.size()==3)
            {
                object=command.get(2).toString();
            }
            else
            {
                reply.addVocab(Vocab::encode("nack"));
                return true;
            }

            PointCloud<DataXYZRGBA> pc;

            if (acquirePointCloud(pc, object))
            {
                if (pc.size()>0)
                {
                    LockGuard lg(mutex);

                    all_points.clear();
                    all_colors.clear();
                    in_points.clear();
                    out_points.clear();
                    dwn_points.clear();

                    Vector p(3);
                    vector<unsigned char> c(3);
                    for (int i=0; i<pc.size(); i++)
                    {
                        p[0]=pc(i).x;
                        p[1]=pc(i).y;
                        p[2]=pc(i).z;
                        c[0]=pc(i).r;
                        c[1]=pc(i).g;
                        c[2]=pc(i).b;
                        all_points.push_back(p);
                        all_colors.push_back(c);
                    }

                    removeOutliers();
                    sampleInliers();

                    vtk_all_points->set_points(all_points);
                    vtk_all_points->set_colors(all_colors);
                    vtk_out_points->set_points(out_points);
                    vtk_dwn_points->set_points(dwn_points);
                }

                computeAndVisualizeSuperqAndGrasp();
            }
        }


        reply.addVocab(Vocab::encode(ok?"ack":"nack"));
        return true;
    }

    /****************************************************************/
    bool acquirePointCloud(PointCloud<DataXYZRGBA> &point_cloud, const string &object)
    {
        //  query point-cloud-read via rpc for the point cloud
        //  command: get_point_cloud objectName
        //  put point cloud into container, return true if operation was ok
        //  or call refreshpointcloud
        Bottle cmd_request;
        Bottle cmd_reply;

        cmd_request.addString("look");
        cmd_request.addString(object);
        cmd_request.addString("wait");

        action_render_rpc.write(cmd_request, cmd_reply);
        if (cmd_reply.toString() != "[ack]")
        {
            yError() << "Didn't manage to look at the object";
            return false;
        }

        point_cloud.clear();
        cmd_request.clear();
        cmd_reply.clear();

        cmd_request.addString("get_point_cloud");
        cmd_request.addString(object);

        point_cloud_rpc.write(cmd_request, cmd_reply);

        //  cheap workaround to get the point cloud
        Bottle* pcBt = cmd_reply.get(0).asList();
        bool success = point_cloud.fromBottle(*pcBt);

        if (success && (point_cloud.size() > 0))
        {
            yDebug() << "Point cloud retrieved; contains " << point_cloud.size() << "points";
            refreshPointCloud(point_cloud);
            return true;
        }
        else
        {
            yError() << "Point cloud null or empty";
            return false;
        }

    }

    /****************************************************************/
    void refreshPointCloud(const PointCloud<DataXYZRGBA> &points)
    {
       if (points.size() > 0)
       {
           LockGuard lg(mutex);

           //   set the vtk point cloud object with the read data
           //vtk_all_points->set_points(points);
           //vtk_all_points->set_colors(points);

           //   position the camera to look at point cloud
           vector<double> bounds(6), centroid(3);
           vtk_all_points->get_polydata()->GetBounds(bounds.data());

           double bb = 0.0;
           for (size_t i=0; i<centroid.size(); i++)
           {
               centroid[i] = 0.5 * (bounds[i<<1] + bounds[(i<<1)+1]);
               bb = std::max(bb, bounds[(i<<1)+1] - bounds[i<<1]);
           }
           bb *= 3.0;

           vtk_camera->SetPosition(centroid[0] + bb, centroid[1], centroid[2] + bb);
           vtk_camera->SetViewUp(0.0, 0.0, 1.0);
           vtk_camera->SetFocalPoint(centroid.data());
       }
    }

    /****************************************************************/
    bool interruptModule() override
    {
        closing=true;
        if (!from_file)
        {
            action_render_rpc.interrupt();
            point_cloud_rpc.interrupt();
        }
        return true;

    }

    /****************************************************************/
    bool close() override
    {
        if (!from_file)
        {
            if (rpcPoints.asPort().isOpen())
                rpcPoints.close();
            if (rpcService.asPort().isOpen())
                rpcService.close();
            if (action_render_rpc.asPort().isOpen())
                action_render_rpc.close();
            if (point_cloud_rpc.asPort().isOpen())
                point_cloud_rpc.close();
        }
        return true;
    }

    /****************************************************************/
    void samplePointsHand(Vector &hand, vector<Vector> &points, string &str_hand)
    {
        double se, ce, co, so;
        double omega;

        Vector point(3,0.0);

        for(int i=0; i<(int)sqrt(36); i++)
        {
            for (double theta=0; theta<=2*M_PI; theta+=M_PI/((int)sqrt(36)))
            {
                if (str_hand=="right")
                {
                    omega=i*M_PI/sqrt(36);

                    ce=cos(theta);
                    se=sin(theta);
                    co=cos(omega);
                    so=sin(omega);

                    point[0]=hand[7] * sign(ce)*(pow(abs(ce),hand[10])) * sign(co)*(pow(abs(co),hand[11]));
                    point[1]=hand[8] * sign(se)*(pow(abs(se),hand[10]));
                    point[2]=hand[9] * sign(ce)*(pow(abs(ce),hand[10])) * sign(so)*(pow(abs(so),hand[11]));
                }
                else
                {
                    omega=i*M_PI/sqrt(36);

                    ce=cos(theta);
                    se=sin(theta);

                    co=cos(omega);
                    so=sin(omega);

                    point[0]=hand[7] * sign(ce)*(pow(abs(ce),hand[10])) * sign(co)*(pow(abs(co),hand[11]));
                    point[1]=hand[8] * sign(se)*(pow(abs(se),hand[10]));
                    point[2]=hand[9] * sign(ce)*(pow(abs(ce),hand[10])) * sign(so)*(pow(abs(so),hand[11]));
                }


                Vector point_tr(4,0.0);

                Matrix H_hand;

                H_hand=axis2dcm(hand.subVector(3,6));
                H_hand.setSubcol(hand.subVector(0,2), 0,3);

                if (str_hand=="right")
                {
                    if (point[0] + point[2] < 0)
                    {
                        Vector point_tmp(4,1.0);
                        point_tmp.setSubvector(0,point);
                        point_tr=H_hand*point_tmp;
                        point=point_tr.subVector(0,2);
                        points.push_back(point);
                    }
                }
                else
                {
                    if (point[0] - point[2] < 0)
                    {
                        Vector point_tmp(4,1.0);
                        point_tmp.setSubvector(0,point);
                        point_tr=H_hand*point_tmp;
                        point=point_tr.subVector(0,2);
                        points.push_back(point);
                    }
                }
            }
        }

        //for (size_t i=0; i<points.size(); i++)
        yDebug()<<"Points on hand"<<points.size();
    }

public:
    /****************************************************************/
    Visualizer() : closing(false) {} //, pointsProcessor(this) { }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (!rf.check("file"))
    {
        if (!yarp.checkNetwork())
        {
            yError()<<"Unable to find Yarp server!";
            return EXIT_FAILURE;
        }
    }

    Visualizer visualizer;
    return visualizer.runModule(rf);
}


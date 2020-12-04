// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/calib3d.hpp"
#include "myslam/config.h"
#include "myslam/visual_odometry.h"

#include <pangolin/pangolin.h>
using namespace pangolin;

using namespace cv;
int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    myslam::Config::setParameterFile ( argv[1] );
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );  // TODO new Class

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

//    /* ================================================使用opencv的viz工具显示3D信息,摄像机Pose============================================================ */
//    // visualization
//    cv::viz::Viz3d vis ( "Visual Odometry" );
//    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
//    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
//    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
//    vis.setViewerPose ( cam_pose );
//
//    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
//    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
//    vis.showWidget ( "World", world_coor );
//    vis.showWidget ( "Camera", camera_coor );
//
//
//    //
//    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
//    for ( int i=0; i<rgb_files.size(); i++ )
//    {
//        cout<<"****** loop "<<i<<" ******"<<endl;
//        Mat color = cv::imread ( rgb_files[i] );
//        Mat depth = cv::imread ( depth_files[i], -1 );
//        if ( color.data==nullptr || depth.data==nullptr )
//            break;
//        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();  // TODO retrun  => Frame::Ptr   createFrame() => return Frame::Ptr( new Frame(factory_id++) );
//        pFrame->camera_ = camera;
//        pFrame->color_ = color;
//        pFrame->depth_ = depth;
//        pFrame->time_stamp_ = rgb_times[i];
//
//        boost::timer timer;
//        vo->addFrame ( pFrame );
//        cout<<"VO costs time: "<<timer.elapsed() <<endl;
//
//        if ( vo->state_ == myslam::VisualOdometry::LOST )
//            break;
//        SE3 Twc = pFrame->T_c_w_.inverse();
//
//        // show the map and the camera pose (by cv viz)
//        cv::Affine3d M (
//            cv::Affine3d::Mat3 (
//                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
//                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
//                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
//            ),
//            cv::Affine3d::Vec3 (
//                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
//            )
//        );
//
//        Mat img_show = color.clone();
//        for ( auto& pt:vo->map_->map_points_ )
//        {
//            myslam::MapPoint::Ptr p = pt.second; // todo  .second => unordered_map<unsigned long, MapPoint::Ptr >  map_points_;
//            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
//            cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
//        }
//        cv::imshow ( "image", img_show );
//        cv::waitKey ( 0 );
//        vis.setWidgetPose ( "Camera", M ); //M(Mat3,Vec3)
//        vis.spinOnce ( 1, false );
//
//        // show the map and the camera pose (by Pangolin)
//
//        cout<<endl;
//    }

    /* ==========================================使用pangolin工具显示3D的信息,摄像机的位姿pose ============================================================= */
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);//深度测试
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(//摆放一个相机
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );
    pangolin::View &d_cam = pangolin::CreateDisplay()//创建一个窗口
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//消除颜色缓冲
        d_cam.Activate(s_cam);

        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
        // draw poses
        //画相机朝向
        SE3 temp;
        cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
        for ( int i=0; i<rgb_files.size(); i++ )
        {
            cout<<"****** loop "<<i<<" ******"<<endl;
            Mat color = cv::imread ( rgb_files[i] );
            Mat depth = cv::imread ( depth_files[i], -1 );
            if ( color.data==nullptr || depth.data==nullptr )
                break;
            myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();  // TODO retrun  => Frame::Ptr   createFrame() => return Frame::Ptr( new Frame(factory_id++) );
            pFrame->camera_ = camera;
            pFrame->color_ = color;
            pFrame->depth_ = depth;
            pFrame->time_stamp_ = rgb_times[i];

            boost::timer timer;
            vo->addFrame ( pFrame );
            cout<<"VO costs time: "<<timer.elapsed() <<endl;

            if ( vo->state_ == myslam::VisualOdometry::LOST )
                break;
            SE3 Twc = pFrame->T_c_w_.inverse();
            // show the map and the camera pose (by Pangolin)
            temp=Twc;
            glPushMatrix();
            Sophus::Matrix4f m = Twc.matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());

            const float w = 0.25;
            const float h = w*0.75;
            const float z = w*0.6;
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            //画相机模型
            glVertex3f(0, 0, 0);
            glVertex3f(w,h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(w,-h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,-h,z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);
            glVertex3f(-w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);
            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);

//            glEnd();
            glPopMatrix();

            glColor3f(0, 0, 1);
            glBegin(GL_LINES);
            auto p1 = temp, p2 = Twc;
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);

            glEnd();
            pangolin::FinishFrame();
//            usleep(500);   // sleep 5 ms
            Mat img_show = color.clone();
            for ( auto& pt:vo->map_->map_points_ )
            {
                myslam::MapPoint::Ptr p = pt.second; // todo  .second => unordered_map<unsigned long, MapPoint::Ptr >  map_points_;
                Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
                cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
            }
            cv::imshow ( "image", img_show );
            cv::waitKey ( 500 );

            cout<<endl;
        }
    }

    return 0;
}






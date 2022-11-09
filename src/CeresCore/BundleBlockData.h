#pragma once
#include "Enumerations.h"
#include "SensorData.h"
#include "CameraData.h"
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>

namespace CeresCore
{
    class BundleBlockData
    {
    public:

        BundleBlockData()
        {};
        ~BundleBlockData()
        {
            delete[] gcp_index_;
            delete[] point_index_;
            delete[] camera_index_;
            delete[] sensor_index_;
            delete[] observations_;
            delete[] parameters_;
        }

        int num_observations()       const { return num_observations_; }
        const double* observations() const { return observations_; }
        const double* gcp_observations() const { return observations_ + num_observations_ * 2; }

        double* mutable_cameras() { return parameters_; }
        double* mutable_cameras(int i) { return mutable_cameras() + 6 * i; }
        double* mutable_sensor() { return parameters_ + 6 * num_cameras_; }
        double* mutable_sensor(int i) { return mutable_sensor() + num_camera_model_param * i; }
        double* mutable_points() { return parameters_ + 6 * num_cameras_ + num_camera_model_param * num_sensors_; }
        double* mutable_points(int i) { return mutable_points() + 3 * i; }

        double* mutable_camera_for_observation(int i) {
            return mutable_cameras() + camera_index_[i] * 6;
        }
        double* mutable_sensor_for_observation(int i) {
            return mutable_sensor() + sensor_index_[i] * num_camera_model_param;
        }
        double* mutable_point_for_observation(int i) {
            return mutable_points() + point_index_[i] * 3;
        }
        /*double* mutable_ls_point_for_observation(int i)
        {
            return lspoints_ + point_index_[i] * 3;
        }*/

        
        bool use_ls_points;
        /*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ls;*/

        int num_sensors_;
        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;
        int num_gcp_;

        double icpDistanceThresh;

        int* gcp_index_;
        int* point_index_;
        int* camera_index_;
        int* sensor_index_;
        double* observations_;
        double* parameters_;
        double* lspoints_;
        bool useICPOrientation;

        const char* icpPointCloudFilename;

        std::vector<CeresCore::SensorData*> sensors_;
        std::vector<CeresCore::CameraData*> cameras_;
        double w_img_, w_gcp_, w_icp_;

        pcl::Correspondences currentIcpMatch;

        void SetCameraModel(CameraModel model)
        {
            camera_model = model;
            switch (camera_model)
            {
            default:
            case CameraModel::Brown:
                num_camera_model_param = 13;
                break;
            case CameraModel::Seidel:
                num_camera_model_param = 17;
                break;
            }
        }
        CameraModel GetCameraModel()
        {
            return camera_model;
        }

        int GetCameraModelNumParameters()
        {
            return num_camera_model_param;
        }

    private:
        CameraModel camera_model;
        int num_camera_model_param = 13;
    };
}
#pragma once
#include <cmath>
#include <cstdio>
#include <iostream>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
//#include "CustomIterationCallback.h"
//#include "SnavelyReprojectionError.h"
#include "CollinearitySiedelError.h"
#include "CollinearityError.h"
#include "GroundPointError.h"
#include "IcpError.h"
#include "IcpError2.h"
#include "BundleBlockData.h"


#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/obj_io.h>
//#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>

namespace CeresCore
{
    typedef ceres::CallbackReturnType (__stdcall* PFOnEventCallback)(ceres::IterationSummary* summary, void* sender);
    class BundleSolver
    {


    public:
        BundleSolver();
        ~BundleSolver();

        ceres::Solver::Options* options() { return &_options; }
        const ceres::Solver::Summary summary() { return _summary; }

        bool SetLinearSolverType(const ceres::LinearSolverType solverType);

        bool Run(BundleBlockData* data);
        bool RunCore();


        const std::string GetSummary(const bool fullSummary);
        const std::vector<double> GetResidual() { return _evaluated_residuals; }
        /*void add_custom_iteration_callback(CeresCore::CustomIterationCallback* callback);*/
        void registerOnEventCallback(PFOnEventCallback newVal) { m_fireEvent = newVal; }
        const std::vector<double> GetParameters();

        PFOnEventCallback m_fireEvent;
        void setWrapperObject(void* ptr) { wrapperObject = ptr; }
        void* getWrapperObject() { return wrapperObject; }
        double* GetPointCloudElement(int id);
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr ReadPointCloud(const std::string& cloud1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr BundleSolver::ReadBlockPointCloud();
        
        void* wrapperObject;
        CeresCore::BundleBlockData* _bundle_data_;

        ceres::Problem _problem;
        ceres::Solver::Options _options;
        ceres::Solver::Summary _summary;
        std::vector<double> _evaluated_residuals;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ls;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ph;
        double* ls_obs;

        void BundleSolver::SetCorrespondences();
       /* void BundleSolver::GetICPMatchingPoints();*/

        class IcpCallback :
            public ceres::IterationCallback
        {
        public:

            IcpCallback(BundleSolver* solver)
            {
                _solver = solver;
            }

            virtual ceres::CallbackReturnType operator()(const
                ceres::IterationSummary& summary)
            {
                _solver->SetCorrespondences();
                return ceres::CallbackReturnType::SOLVER_CONTINUE;
            }

        private:
            CeresCore::BundleSolver* _solver;
        };

        class IterCallback :
            public ceres::IterationCallback
        {
        public:

            IterCallback(BundleSolver* solver)
            {
                _solver = solver;
            }

            virtual ceres::CallbackReturnType operator()(const
                ceres::IterationSummary& summary)
            {
                ceres::CallbackReturnType result = ceres::CallbackReturnType::SOLVER_CONTINUE;
                ceres::IterationSummary* sum = new ceres::IterationSummary(summary);
                if (_solver->m_fireEvent)
                    ceres::CallbackReturnType result = _solver->m_fireEvent(sum, _solver->wrapperObject);
                return result;
            }

        private:
            CeresCore::BundleSolver* _solver;
        };
    };

    
}
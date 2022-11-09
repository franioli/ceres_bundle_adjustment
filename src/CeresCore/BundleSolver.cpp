#include "BundleSolver.h"
#include <iostream>
//#include "..\PCLCore\Test.h"

namespace CeresCore
{
    double IcpError::dist_thresh;
	BundleSolver::BundleSolver()
	{
		std::cout << "Created the BundleSolver object!" << std::endl;

        _options = ceres::Solver::Options();
        _options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
        _options.minimizer_progress_to_stdout = true;
        _options.max_num_iterations = 1;
        _options.update_state_every_iteration = true;

        CeresCore::BundleSolver::IterCallback* iterCallback = new CeresCore::BundleSolver::IterCallback(this);
        _options.callbacks.push_back(iterCallback);
        m_fireEvent = nullptr;
	}

	BundleSolver::~BundleSolver()
	{
	}

    bool BundleSolver::Run(BundleBlockData* bundle_data)
    {
        _bundle_data_ = bundle_data;
        //CeresCore::Prova * prova = new CeresCore::Prova();
        return RunCore();
    }

    void BundleSolver::SetCorrespondences()
    {
        pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
        cloud_ph = ReadBlockPointCloud();
        est.setInputSource(cloud_ph);
        est.setInputTarget(cloud_ls);
        pcl::Correspondences matches;

        // la distanza utilizzata qui è quella reale, quella memorizzata nella classe pcl::Correspondence è 
        // la distanza al quadrato (vd. file correspondence_estimation.hpp riga 123, 139 e 144
        est.determineCorrespondences(matches, _bundle_data_->icpDistanceThresh); 
        _bundle_data_->currentIcpMatch = matches;

        for (size_t i = 0; i < _bundle_data_->num_points_; i++)
        {
            ls_obs[3 * i] = std::numeric_limits<double>::quiet_NaN();
            ls_obs[3 * i + 1] = std::numeric_limits<double>::quiet_NaN();
            ls_obs[3 * i + 2] = std::numeric_limits<double>::quiet_NaN();
        }

        double sum = 0.0;
        int count = 0;
        for (size_t i = 0; i < matches.size(); i++)
        {
            pcl::Correspondence match = matches[i];
            int ptId = match.index_query;
            int lsId = match.index_match;
            pcl::PointXYZ pt = cloud_ls->points[lsId];
            /*double* X = _bundle_data_->mutable_point_for_observation(ptId);
            double dx = pt.x - X[0];
            double dy = pt.y - X[1];
            double dz = pt.z - X[2];
            double dist = dx * dx + dy * dy + dz * dz;
            if (dist > 4)
                continue;*/
            ls_obs[3 * ptId] = pt.x;
            ls_obs[3 * ptId + 1] = pt.y;
            ls_obs[3 * ptId + 2] = pt.z;
            // la distanza memorizzata nella classe pcl::Correspondence è 
            // la distanza al quadrato (vd. file correspondence_estimation.hpp riga 123, 139 e 144
            sum += std::sqrt(match.distance);
            count++;
        }
        std::cout << "Match size: " << matches.size() << "( " << count << " )" << "\tAvg. Distance: " << sum / count << std::endl;
    }

    bool BundleSolver::RunCore()
    {
        if (_bundle_data_->useICPOrientation)
        {
            cloud_ls = ReadPointCloud(std::string(_bundle_data_->icpPointCloudFilename));
            /*_bundle_data_->cloud_ls = cloud_ls;*/

            ls_obs = new double[3 * _bundle_data_->num_points_];
            SetCorrespondences();
            
        }
        const double* obs = _bundle_data_->observations();
        const double* gcp_obs = _bundle_data_->gcp_observations();
        // Create residuals for each observation in the bundle adjustment problem. The
        // parameters for cameras and points are added automatically.
        std::vector<ceres::ResidualBlockId> to_eval;


        for (int i = 0; i < _bundle_data_->num_observations(); ++i)
        {
            // Each Residual block takes a point and a camera as input and outputs a 2
            // dimensional residual. Internally, the cost function stores the observed
            // image location and compares the reprojection against the observation.
            ceres::CostFunction* cost_function;
            switch (_bundle_data_->GetCameraModel())
            {
            case CameraModel::Seidel:
                cost_function =
                    CollinearitySiedelError::CreateAutoDiff(obs[2 * i + 0],
                        obs[2 * i + 1]);
                break;
            case CameraModel::Brown:
            default:
                cost_function =
                    CollinearityError::CreateAutoDiff(obs[2 * i + 0],
                        obs[2 * i + 1]);
                break;
            }
            /*ceres::ScaledLoss loss(nullptr, 1.0, ceres::TAKE_OWNERSHIP);*/
            ceres::LossFunction* loss = new ceres::ScaledLoss(nullptr, _bundle_data_->w_img_, ceres::TAKE_OWNERSHIP);
            ceres::ResidualBlockId r_id = _problem.AddResidualBlock(cost_function,
                loss,
                _bundle_data_->mutable_sensor_for_observation(i),
                _bundle_data_->mutable_camera_for_observation(i),                              
                _bundle_data_->mutable_point_for_observation(i));
            to_eval.push_back(r_id); 
        }
        
        for (size_t i = 0; i < _bundle_data_->num_gcp_; i++)
        {
            ceres::CostFunction* cost_function =
                GroundPointError::CreateAutoDiff(gcp_obs[3 * i + 0],
                    gcp_obs[3 * i + 1], gcp_obs[3 * i + 2]);
            ceres::LossFunction* loss = new ceres::ScaledLoss(nullptr, _bundle_data_->w_gcp_, ceres::TAKE_OWNERSHIP);

            ceres::ResidualBlockId r_id = _problem.AddResidualBlock(cost_function,
                loss,
                _bundle_data_->mutable_points(_bundle_data_->gcp_index_[i]));
            to_eval.push_back(r_id);
        }

        if (_bundle_data_->useICPOrientation) {
            IcpError::dist_thresh = _bundle_data_->icpDistanceThresh;
            for (size_t i = 0; i < _bundle_data_->num_points_; i++)
            {
                ceres::CostFunction* cost_function =
                    IcpError::CreateAutoDiff(ls_obs + 3 * i,
                        ls_obs + 3 * i + 1, ls_obs + 3 * i + 2);
                ceres::LossFunction* loss = new ceres::ScaledLoss(nullptr, _bundle_data_->w_icp_, ceres::TAKE_OWNERSHIP);

                ceres::ResidualBlockId r_id = _problem.AddResidualBlock(cost_function,
                    loss,
                    _bundle_data_->mutable_points(i));
                to_eval.push_back(r_id);
            }
            CeresCore::BundleSolver::IcpCallback* icpCallback = new CeresCore::BundleSolver::IcpCallback(this);
            _options.callbacks.push_back(icpCallback);
        }

        for (size_t i = 0; i < _bundle_data_->sensors_.size(); i++)
        {
            ceres::SubsetParameterization* subset_parameterization = new ceres::SubsetParameterization(_bundle_data_->GetCameraModelNumParameters(), _bundle_data_->sensors_[i]->fixed_parameters);
            _problem.SetParameterization(_bundle_data_->mutable_sensor(i), subset_parameterization);
        }

        for (size_t i = 0; i < _bundle_data_->num_cameras_; i++)
        {
            if (_bundle_data_->cameras_[i]->fixed_parameters.size() == 0)
                continue;
            ceres::SubsetParameterization* subset_parameterization = new ceres::SubsetParameterization(6, _bundle_data_->cameras_[i]->fixed_parameters);
            _problem.SetParameterization(_bundle_data_->mutable_cameras(i), subset_parameterization);
        }
       /* _problem.SetParameterBlockConstant(_bundle_data_->mutable_sensor_for_observation(0));*/
        

        // Make Ceres automatically detect the bundle structure. Note that the
        // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
        // for standard bundle adjustment problems.
        /*ceres::Solver::Options options;*/
        
        ceres::Solve(_options, &_problem, &_summary);

        double total_cost = 0.0;
        ceres::Problem::EvaluateOptions options;
        options.residual_blocks = to_eval;
        _problem.Evaluate(options, &total_cost, &_evaluated_residuals, nullptr, nullptr);
        //if (_bundle_data_->useICPOrientation) {
        //    GetICPMatchingPoints();
        //}
        return true;
    }

    //void BundleSolver::GetICPMatchingPoints()
    //{

    //}
    pcl::PointCloud<pcl::PointXYZ>::Ptr BundleSolver::ReadPointCloud(const std::string& file1)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        std::string ext = file1.substr(file1.find_last_of(".") + 1);
        std::for_each(ext.begin(), ext.end(), [](char& c) { c = ::tolower(c); });
        int result = -1;
        if (ext == "pcd")
            result = pcl::io::loadPCDFile<pcl::PointXYZ>(file1, *temp);
        else if (ext == "obj")
        {
            result = pcl::io::loadOBJFile<pcl::PointXYZ>(file1, *temp);
            std::string file2 = file1.substr(0, file1.find_last_of('.')) + ".pcd";
            result = pcl::io::savePCDFileBinary<pcl::PointXYZ>(file2, *temp);
        }
        //else if (ext == "ply")
        //    result = pcl::io::loadPLYFile<pcl::PointXYZ>(file1, *temp);


        /*int result = pcl::io::loadOBJFile<pcl::PointXYZ>(file1, *temp);
        result = pcl::io::savePCDFileBinary<pcl::PointXYZ>(file1, *temp);*/
        std::cout << "Saved " << temp->size() << " data points to input:" << std::endl;

        return temp;
    }
    double* BundleSolver::GetPointCloudElement(int id)
    {
        pcl::PointXYZ pt = cloud_ls->points[id];
        double* v = new double[] {pt.x, pt.y, pt.z};
        
        return v;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr BundleSolver::ReadBlockPointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < _bundle_data_->num_points_; ++i)
        {
            double* X = _bundle_data_->mutable_points(i);
            pcl::PointXYZ pt((float)X[0], (float)X[1], (float)X[2]);
            temp->push_back(pt);
        }
        return temp;
    }
    const std::vector<double> BundleSolver::GetParameters()
    {
        return std::vector<double>();
    }

    //void BundleSolver::add_custom_iteration_callback(CustomIterationCallback* callback)
    //{
    //    /*_options.callbacks.push_back(callback);*/
    //}

    

    bool BundleSolver::SetLinearSolverType(const ceres::LinearSolverType solverType)
    {
        _options.linear_solver_type = solverType;
        return true;
    }

    const std::string BundleSolver::GetSummary(const bool fullSummary)
    {
        if (fullSummary)
            return _summary.FullReport();
        else
            return _summary.BriefReport();
    }

}
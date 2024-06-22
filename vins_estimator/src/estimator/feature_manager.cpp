/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "feature_manager.h"
#include "../cpnp_solver/cpnp.h"
#include "Eigen/src/Core/Matrix.h"
#include "Eigen/src/Core/util/Memory.h"
#include "opengv/types.hpp"
#include "parameters.h"
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include "opengv/absolute_pose/methods.hpp"
#include "opengv/absolute_pose/CentralAbsoluteAdapter.hpp"

#include "parameters.h"
#include <string>

static Eigen::Matrix3d r_cpnp, r_iter, r_sqpnp;
static Eigen::Vector3d t_cpnp, t_iter, t_sqpnp;

int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs), first_flag(true)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

void FeatureManager::clearState()
{
    feature.clear();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}


void FeatureManager::addFeatureToIdPts(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    last_track_num = 0;
    new_feature_num = 0;
    long_track_num = 0;
    /*
    img的结构:      first(key) - second(vec):{ first(pair 1)   second(pair 2) }
    <key-value> : <feat_id_per_frame ,   vector<cam_id , feat_info> >
    feat_id_per_frame用于获取左右相机的特征点的对应关系, cam_id:0表示左相机, cam_id:1表示右相机
     */
    for (auto &id_pts : image)
    {
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td); // 创建特征点在当前帧的信息
        assert(id_pts.second[0].first == 0); 
        // 左目的一些点只和上一帧左目有对应, 因此左i和左i+1 与 左i+1和右i+1 之间的对应点关系不同
        if(id_pts.second.size() == 2) // 如果该特征点在右图也有对应, 则vector里有2个pair
        {
            f_per_fra.rightObservation(id_pts.second[1].second); // 记录右图的观测信息
            assert(id_pts.second[1].first == 1); // 确保是右图的特征点
        }

        // 获取特征点滑窗id,用于在多帧图像中进行追踪
        // 不是当前帧id(用于左右目对应)
        int feature_id = id_pts.first; 
        
        // 查找当前特征点是否已经被跟踪(用于维护滑窗)
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
            return it.feature_id == feature_id; 
                          });

        if (it == feature.end()) // 如果当前特征点未被跟踪
        {
            feature.push_back(FeaturePerId(feature_id, frame_count)); // 添加新的特征点, 记录id和起始帧
            feature.back().feature_per_frame.push_back(f_per_fra); // 记录特征点在当前帧的信息
            new_feature_num++;
        }
        else if (it->feature_id == feature_id) // 如果当前特征点已经被跟踪
        {
            it->feature_per_frame.push_back(f_per_fra); // 更新特征点信息
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4) // 如果特征点被连续跟踪的帧数超过4
                long_track_num++; // 长时间跟踪 的特征点数量加一, @todo 后面用于判断是否需要进行三角化
        }
    }
}


void FeatureManager::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


void FeatureManager::initFramePoseByPnP(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[],double header_time)
{
    if(frameCnt > 0)
    {
        vector<cv::Point2f>       pts2D;
        vector<cv::Point3f>       pts3D;
        vector<Eigen::Vector3d, Eigen::aligned_allocator<Vector3d>> epts_3d,epts_norm_2d;
        vector<Eigen::Vector2d> epts_2d;
        pts2D.reserve(1500);
        epts_norm_2d.reserve(1500);
        pts3D.reserve(1500);
        epts_3d.reserve(1500);
         epts_2d.reserve(1500);

        for (auto &it_per_id : feature)
        {
            if (it_per_id.estimated_depth > 0)
            {
                // 青睐多帧观测的特征点
                int index = frameCnt - it_per_id.start_frame;
                if((int)it_per_id.feature_per_frame.size() >= index + 1)
                {
                    Vector3d ptsInCam = ric[0] * (it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth) + tic[0];
                    Vector3d ptsInWorld = Rs[it_per_id.start_frame] * ptsInCam + Ps[it_per_id.start_frame];

                    cv::Point3f point3d(ptsInWorld.x(), ptsInWorld.y(), ptsInWorld.z());
                    cv::Point2f point2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y());
                    pts3D.push_back(point3d);
                    pts2D.push_back(point2d); 

                    epts_3d.push_back(ptsInWorld);
                    epts_norm_2d.push_back(it_per_id.feature_per_frame[index].point);
                    epts_2d.push_back(Eigen::Vector2d(it_per_id.feature_per_frame[index].point.x(), it_per_id.feature_per_frame[index].point.y()));
                }
            }
        }


        Eigen::Matrix3d RCam;
        Eigen::Vector3d PCam;

        cv::Mat R_cv, rvec, tvec;    

        std::vector<int> inliers;
        cv::Mat cam_mat = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);  
        if(cv::solvePnPRansac(pts3D, pts2D, cam_mat, cv::noArray(), rvec, tvec, false, 2000, 0.05, 0.995, inliers, cv::SOLVEPNP_ITERATIVE)) 
        {
            
            // Filter epts_3d and epts_2d based on inliers  
            vector<Eigen::Vector3d , Eigen::aligned_allocator<Eigen::Vector3d>> filtered_epts_3d, filter_norm;
            vector<Eigen::Vector2d> filtered_epts_2d;
            vector<cv::Point2f> cvp2;
            vector<cv::Point3f> cvp3;
            filtered_epts_2d.reserve(inliers.size());
            filter_norm.reserve(inliers.size());
            filtered_epts_3d.reserve(inliers.size());
//             inliers保存了inlier的索引(不是mask!)
            for (int idx : inliers) {
                filtered_epts_3d.emplace_back(epts_3d[idx]);
                filtered_epts_2d.emplace_back(epts_2d[idx]);
                filter_norm.emplace_back(epts_norm_2d[idx]);
                cvp2.push_back(pts2D[idx]);
                cvp3.push_back(pts3D[idx]);
            }
            std::cout<<"{inlier num}:"<< inliers.size() <<std::endl;
            std::cout<<"{outlier}:"<< epts_2d.size()-inliers.size() <<std::endl;

            Eigen::Matrix3d R_i_sq;
            Eigen::Vector3d P_i_sq;
            TicToc t_pnp;
            t_pnp.tic();
            
            // SQPnP
            // cv::solvePnP(cvp3, cvp2, cam_mat, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_SQPNP);
            // cv::Rodrigues(rvec, R_cv);
            // cv::cv2eigen(R_cv,RCam);
            // cv::cv2eigen(tvec,PCam);
            
            // iterative
            // cv::solvePnP(cvp3, cvp2, cam_mat, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            // cv::Rodrigues(rvec, R_cv);
            // cv::cv2eigen(R_cv,RCam);
            // cv::cv2eigen(tvec,PCam);

            // CPnP
            pnpsolver::CPnP(filtered_epts_2d, filtered_epts_3d, CAM_PARAM, RCam, PCam);

            // 转换到i到i+1坐标系下, opencv的solvePnP是将3D点(cam_i)转到相机坐标系下的(cam_i+1)
            // 里程计的转换相反
            RCam.transposeInPlace();
            PCam = -RCam * PCam;
            // 转到世界坐标系下
            R_i_sq = RCam * ric[0].transpose(); 
            P_i_sq = -RCam * ric[0].transpose() * tic[0] + PCam;

            // MLPnP, 点数必须大于6否则触发exception.
            // if(inliers.size()>6)
            // {
            //     opengv::absolute_pose::CentralAbsoluteAdapter ad(filter_norm,filtered_epts_3d);
            //     opengv::transformation_t pose = opengv::absolute_pose::mlpnp(ad);
            //     RCam = pose.block<3, 3>(0, 0);
            //     PCam = pose.block<3, 1>(0, 3);
            // R_i_sq = RCam * ric[0].transpose(); 
            // P_i_sq = -RCam * ric[0].transpose() * tic[0] + PCam;
            // }
            // else // 小于6直接用上一帧的位姿
            // {
            //     R_i_sq = Rs[frameCnt - 1];
            //     P_i_sq = Ps[frameCnt - 1];
            // }

            double dur = t_pnp.toc();
            std::ofstream foutime(OUTPUT_FOLDER+"/solve_time.csv" , std::ios::app);
            foutime.setf(ios::fixed, ios::floatfield);
            foutime << dur << endl;
            foutime.close();

            std::ofstream fouti(OUTPUT_FOLDER+"/inlier_"+std::to_string(MAX_CNT)+".csv" , std::ios::app);
            fouti.setf(ios::fixed, ios::floatfield);
            fouti << inliers.size() << endl;
            fouti.close();

            Rs[frameCnt] = R_i_sq;
            Ps[frameCnt] = P_i_sq;
        }
    }
}

void FeatureManager::triangulate(int frameCnt, Vector3d Ps[], Matrix3d Rs[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature) // feature在之前的addFeatureCheckParallax()中已经被更新/初始化
    {
        if (it_per_id.estimated_depth > 0) // 有深度估计的点跳过
            continue;
        
        if(it_per_id.feature_per_frame[0].is_stereo) // 该点是左右目都有观测的点
        {
            // 提取左右相机的位姿并转换到世界坐标系下进行三角化
            int idx = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[idx] + Rs[idx] * tic[0];
            Eigen::Matrix3d R0 = Rs[idx] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;
            //cout << "left pose " << leftPose << endl;

            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[idx] + Rs[idx] * tic[1];
            Eigen::Matrix3d R1 = Rs[idx] * ric[1];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;
            //cout << "right pose " << rightPose << endl;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[0].pointRight.head(2);

            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>(); // 存在左目坐标系下(3d)
            double depth = localPoint.z();

            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            continue;
        }
        else if(it_per_id.feature_per_frame.size() > 1) // 至少连续2次左目有观测的点可以根据之前PnP的结果进行三角化
        {
            // 提取左右相机的位姿并转换到世界坐标系下进行三角化
            int obs_idx = it_per_id.start_frame;
            Eigen::Matrix<double, 3, 4> leftPose;
            Eigen::Vector3d t0 = Ps[obs_idx] + Rs[obs_idx] * tic[0];
            Eigen::Matrix3d R0 = Rs[obs_idx] * ric[0];
            leftPose.leftCols<3>() = R0.transpose();
            leftPose.rightCols<1>() = -R0.transpose() * t0;

            obs_idx++;
            Eigen::Matrix<double, 3, 4> rightPose;
            Eigen::Vector3d t1 = Ps[obs_idx] + Rs[obs_idx] * tic[0];
            Eigen::Matrix3d R1 = Rs[obs_idx] * ric[0];
            rightPose.leftCols<3>() = R1.transpose();
            rightPose.rightCols<1>() = -R1.transpose() * t1;

            Eigen::Vector2d point0, point1;
            Eigen::Vector3d point3d;
            point0 = it_per_id.feature_per_frame[0].point.head(2);
            point1 = it_per_id.feature_per_frame[1].point.head(2);
            triangulatePoint(leftPose, rightPose, point0, point1, point3d);
            Eigen::Vector3d localPoint;
            localPoint = leftPose.leftCols<3>() * point3d + leftPose.rightCols<1>();
            double depth = localPoint.z();
            if (depth > 0)
                it_per_id.estimated_depth = depth;
            else
                it_per_id.estimated_depth = INIT_DEPTH;
            continue;
        }
/*---------------------------------------------------------------------------------------------*/
        // 实际上如果多帧都观测到了该点, 则可以使用SVD求最小二乘解, 不过这里没用
        // 这是open-vins的做法
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        Eigen::Matrix<double, 3, 4> P0;
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;

            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;
            Eigen::Vector3d f = it_per_frame.point.normalized();
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;
        }
        ROS_ASSERT(svd_idx == svd_A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double svd_method = svd_V[2] / svd_V[3];
        it_per_id.estimated_depth = svd_method;
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;
        }

    }
}

void FeatureManager::removeOutlier(set<int> &outlierIndex)
{
    std::set<int>::iterator itSet;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        int index = it->feature_id;
        itSet = outlierIndex.find(index);
        if(itSet != outlierIndex.end())
        {
            feature.erase(it);
            //printf("remove outlier %d \n", index);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);
                double dep_j = pts_j(2);
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;
            }
        }
    }
}

void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
            it->start_frame--;
        else
        {
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}
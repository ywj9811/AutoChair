#define __BUNDLE_ADJUSTMENT__ 
#include "bundle_adjustment.hpp"

int main() {
    const char* input = "../data/image_formation%d.xyz";
    int input_num = 5;
    double f = 1000, cx = 320, cy = 240;

    cv::viz::Viz3d window("display");
    window.setBackgroundColor();
    window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    window.setWindowSize(cv::Size(1500, 1500));
    window.setWindowPosition(cv::Point(150, 150));
    std::vector<cv::Point3d> point_cloud;
    std::vector<cv::Point3d> point_cloud_before;

    // Load 2D points observed from multiple views;
    std::vector<std::vector<cv::Point2d>> xs;
    for (int i=0; i<input_num; i++) {
        FILE* fin = fopen(cv::format(input, i).c_str(), "rt");
        if (fin == NULL) return -1;
        std::vector<cv::Point2d> pts;
        while (!feof(fin)) {
            double x, y, w;
            if (fscanf(fin, "%lf %lf %lf", &x, &y, &w) == 3)
                pts.push_back(cv::Point2d(x, y));
        }
        fclose(fin);
        xs.push_back(pts);
        if (xs.front().size() != xs.back().size()) return -1;
    }

    // Assumption
    // - All cameras have the same known camera matrix
    // - All points are visible on all camera views
    
    // 1) select the best pair (skipped because all points are visible on all images)
    // 2) Estimate relative pose of the initial two views (epipolar geometry)
    cv::Mat F = cv::findFundamentalMat(xs[0], xs[1], cv::FM_8POINT);
    cv::Mat K = (cv::Mat_<double>(3, 3) << f, 0, cx, 0, f, cy, 0, 0, 1);
    // calculate Essential Matrix
    cv::Mat E = K.t() * F * K, R, t;

    // Single Value Decompostion
    cv::recoverPose(E, xs[0], xs[1], K, R, t);

    std::vector<cv::Vec6d> cameras(xs.size());
    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    cameras[1] = (rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2), t.at<double>(0), t.at<double>(1), t.at<double>(2));

    // 3) Reconstruct 3D points of the initial two views (triangulation)
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    cv::Mat P0 = K * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P1 = K * Rt, X;
    cv::triangulatePoints(P0, P1, xs[0], xs[1], X);

    // Projection [x, y, z , 1]
    std::vector<cv::Point3d> Xs(X.cols);
    X.row(0) = X.row(0) / X.row(3);
    X.row(1) = X.row(1) / X.row(3);
    X.row(2) = X.row(2) / X.row(3);
    X.row(3) = 1;


    // make point cloud 
    for (int c=0; c<X.cols; c++) {
        Xs[c] = cv::Point3d(X.col(c).rowRange(0, 3));
    }

    for(int c=0; c<Xs.size(); c++) {
        cv::Point3d p(
            Xs[c].x,
            Xs[c].y,
            Xs[c].z);
        point_cloud_before.push_back(p);
    }

    // Push constraints of two views
    // Bundle adjustment
    ceres::Problem ba;
    for (size_t j=0; j<2; j++) {
        for (size_t i=0; i<xs[j].size(); i++) {
            ceres::CostFunction* cost_func = ReprojectionError::create(xs[j][i], f, cv::Point2d(cx, cy));
            double* camera = (double*)(&(cameras[j]));
            double* X = (double*)(&(Xs[i]));
            ba.AddResidualBlock(cost_func, NULL, camera, X);
        }
    }

    // Incremntally add more views
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    for (size_t j=2; j<xs.size(); j++) {
        // 4) Select the next image to add (skipped beacause all points are visible on all images)
        // 5) Estimate relative pose of the next view (PnP)
        cv::solvePnP(Xs, xs[j], K, cv::noArray(), rvec, t);
        cameras[j] = (rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2), t.at<double>(0), t.at<double>(1), t.at<double>(2));

        // 6) Reconstruct newly observed 3D points (triangulations)
        // 7) Optimize camera pose and 3D points together (bundle adjustment)
        for (size_t i=0; i<xs[j].size(); i++) {
            ceres::CostFunction* cost_func = ReprojectionError::create(xs[j][i], f, cv::Point2d(cx, cy));
            double* camera = (double*)(&(cameras[j]));
            double* X = (double*)(&(Xs[i]));
            ba.AddResidualBlock(cost_func, NULL, camera, X);
        }

        ceres::Solve(options, &ba, &summary);
    }
    // print summary
    std::cout << summary.FullReport() << std::endl;

    std::cout << "Xs size = " << Xs.size() << std::endl;

    // 3D visualization
    for(int c=0; c<Xs.size(); c++) {
        cv::Point3d p(
            Xs[c].x,
            Xs[c].y,
            Xs[c].z);
        point_cloud.push_back(p);
    }

    cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::green());
    cv::viz::WCloud cloud_widget_before(point_cloud_before, cv::viz::Color::yellow());
    window.showWidget("point_cloud", cloud_widget);
    window.showWidget("point_cloud_before", cloud_widget_before);
    window.spin();

    return 0;
}

#define __BUNDLE_ADJUSTMENT__
#include "bundle_adjustment.hpp"
#include "opencv2/viz.hpp" 

int main(){
    const char* input = "../data/image_formation%d.xyz";
    int input_num = 5;
    double f = 1000, cx = 320, cy = 240;

    cv::viz::Viz3d window("display");
    window.setBackgroundColor();
    window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
    window.setWindowSize(cv::Size(1500, 1500));
    window.setWindowPosition(cv::Point(150, 150));
    std::vector<cv::Point3d> point_cloud;

    // Load 2D points observed from multiple views
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

    // Assumtion 
    // - All cameras have the same and known camera matrix 
    // - All points are visible on all camera views

    // Initalize camera and 3D points 
    std::vector<cv::Vec6d> cameras(xs.size());
    std::vector<cv::Point3d> Xs(xs.front().size(), cv::Point3d(0, 0, 5.5));

    // Optimize camera pose and 3D points together (bundle adjustment)
    ceres::Problem ba;
    for (size_t j=0; j<xs.size(); j++) {
        for (size_t i=0; i<xs[j].size(); i++) {
            ceres::CostFunction* cost_func = ReprojectionError::create(xs[j][i], f, cv::Point2d(cx, cy));
            double* camera = (double*)(&(cameras[j]));
            double* X = (double*)(&(Xs[i]));
            ba.AddResidualBlock(cost_func, NULL, camera, X);
        }
    }
    
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.num_threads = 8;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &ba, &summary);

    for (size_t i=0; i<Xs.size(); i++) {
        printf("%f %f %f\n", Xs[i].x, Xs[i].y, Xs[i].z);
    }

    std::cout << "Xs size " << Xs.size() << std::endl;
    // 3D visualization
    for(int c=0; c<Xs.size(); c++) {
        cv::Point3d p(
            Xs[c].x,
            Xs[c].y,
            Xs[c].z);
        point_cloud.push_back(p);
    }

    cv::viz::WCloud cloud_widget(point_cloud, cv::viz::Color::green());
    window.showWidget("point_cloud", cloud_widget);
    window.spin();
    

    return 0;
}

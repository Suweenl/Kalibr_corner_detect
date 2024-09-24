
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iterator>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include "nlohmann/json.hpp"
// April tags detector and various families that can be selected by command line option
#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"
#include "cameras/GridCalibrationTargetAprilgrid.hpp"

namespace fs = boost::filesystem;
using json = nlohmann::json;

void thresholdMatrix(Eigen::MatrixXd &cornerPoint, double threshold, Eigen::MatrixXd &threshold_mat)
{
    threshold_mat = (cornerPoint.array() > threshold).cast<double>();
    // std::cout << "Processed matrix : \n" << threshold_mat <<std::endl;
}

bool write_to_json(std::string json_path, Eigen::MatrixXd conerPoints, Eigen::MatrixXd points_3d)
{
    json d;
    // d['keypoints3d']=points_3d;
    // d['keypoints2d']=conerPoints;
    d["keypoints3d"] = json::array();
    for (int i = 0; i < points_3d.rows(); ++i)
    {
        // std::cout << "**********@******" << std::endl;
        json point_3 = {points_3d(i, 0), points_3d(i, 1), points_3d(i, 2)};
        d["keypoints3d"].push_back(point_3);
    }
    // std::cout << "*********" <<std::endl;

    d["keypoints2d"] = json::array();
    for (int i = 0; i < conerPoints.rows(); ++i)
    {
        // 检查 conerPoints(i,0) 和 conerPoints(i,1) 是否都小于 1e-9
        if (conerPoints(i, 0) < 1e-9 && conerPoints(i, 1) < 1e-9)
        {
            // 如果是，将第三个元素设置为 0
            json point_2 = {conerPoints(i, 0), conerPoints(i, 1), 0};
            d["keypoints2d"].push_back(point_2);
        }
        else
        {
            // 否则，将第三个元素设置为 1
            json point_2 = {conerPoints(i, 0), conerPoints(i, 1), 1};
            d["keypoints2d"].push_back(point_2);
        }
    }
    // out put
    std::ofstream fout(json_path);
    if (!fout.is_open())
    {
        return false;
    }
    fout << d.dump(4) << std::endl;
    fout.close();
}

std::string get_json_path(std::string image_path)
{
    size_t last_slash_pos = image_path.rfind('/');
    std::string filename = image_path.substr(last_slash_pos + 1);
    size_t dot_pos = filename.rfind('.');
    if (dot_pos == std::string::npos)
    {
        std::cout << "Error: No extension found in the path." << std::endl;
        return image_path;
    }
    std::string number_str = filename.substr(0, dot_pos);
    int number;
    try
    {
        number = std::stoi(number_str);
    }
    catch (...)
    {
        std::cout << "Error: Invalid number in the filename." << std::endl;
        return image_path;
    }
    std::stringstream ss;
    ss << std::setw(7) << std::setfill('0') << number;
    std::string padded_number = ss.str();
    std::string modified_path = image_path.substr(0, last_slash_pos + 1) + padded_number + ".json";
    std::cout << "json_path is " << modified_path << std::endl;
    return modified_path;
}

Eigen::MatrixXd single_detection(std::string image_path, int tagRows, int tagCols, double tagSize, double tagSpacing, bool save_result = false)
{
    cv::Mat image = cv::imread(image_path);
    cv::Mat gray_image;
    std::vector<bool> validCorners;
    std::cout << "load image from : " << image_path << std::endl;
    Eigen::MatrixXd cornerPoints;
    Eigen::MatrixXd threshold_mat;
    if (image.empty())
    {
        std::cerr << "Failed to load image !" << std::endl;
        // return 0;
    }
    // std::string json_path = get_json_path(image_path);
    // RGB to Gray
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
    // detect corner from image
    cameras::GridCalibrationTargetAprilgrid demo(tagRows, tagCols, tagSize, tagSpacing);
    bool result = demo.computeObservation(gray_image, cornerPoints, validCorners);
    if (save_result)
    {
        // std::cout << demo.points()<<std::endl;
        // write to json
        if (cornerPoints.rows() == 0 && cornerPoints.cols() == 0)
        {
            // 则thresold的维度就为大小为（1，tagrows,tagcols）的零矩阵
            cornerPoints.resize(tagRows * tagCols * 4, 2);
            cornerPoints.setZero();
        }
        // if (!fs::exists(json_path))
        // {
        //     bool json_result = write_to_json(json_path, cornerPoints, demo.points());
        // }
        // else
        // {
        //     std::cout << " file exist " << std::endl;
        // }
    }
    else
    {
        std::cout << "Error: No extension found in the path." << std::endl;
    }

    return cornerPoints;
}

void multi_detection(std::string image_folder, int tagRows, int tagCols,
                     double tagSize, double tagSpacing, bool save_result = false)
{
    std::vector<std::string> path_list;
    std::vector<fs::path> fileNames;
    fs::path folder_path(image_folder);
    fs::directory_iterator end_itr; // 尾迭代器
    for (fs::directory_iterator itr(folder_path); itr != end_itr; ++itr)
    {
        if (fs::is_directory(itr->status()))
        {
            std::cout << itr->path().string() << std::endl;
            path_list.push_back(itr->path().string());
        }
    }
    // std::sort(path_list[0].begin(), path_list[0].end());
    // 取四张照片
    for (fs::directory_iterator itr(path_list[0]); itr != end_itr; ++itr)
    {
        // std::cout << "1" <<std::endl;
        Eigen::MatrixXd mat(tagRows * tagCols * 4, 1); // store 0 or 1 for cameras. if 0: detect failed if 1 :detect success
        mat.setZero();
        if (itr->path().extension() == ".jpg" || itr->path().extension() == ".png")
        {
            // 取文件名
            std::string target_name = itr->path().filename().string();
            std::cout << "target name is :" << target_name << std::endl;
            for (const auto &element : path_list)
            {
                std::string target_path = element + "/" + target_name;
                fs::path p(target_path);
                if (!fs::exists(p))
                {
                    std::cout << "lose file :" << target_path << std::endl;
                    continue;
                }
                //  star_detect
                Eigen::MatrixXd threshold_mat;
                std::string json_path = get_json_path(target_path);
                if (fs::exists(json_path)){ 
                    std::cout << " file exitst" << json_path <<std::endl;
                    continue;}
                Eigen::MatrixXd conerpoints = single_detection(target_path, tagRows, tagCols, tagSize, tagSpacing, save_result);
            //     thresholdMatrix(conerpoints, 1e-9, threshold_mat);
            //     // add 0 or 1 into mat .
            //     if (threshold_mat.rows() == 0 && threshold_mat.cols() == 0)
            //     {
            //         // 则thresold的维度就为大小为（1，tagrows,tagcols）的零矩阵
            //         threshold_mat.resize(tagRows * tagCols * 4, 1);
            //         threshold_mat.setZero();
            //     }
            //     // 把thresold 按列与mat向量合并起来
            //     mat.conservativeResize(mat.rows(), mat.cols() + threshold_mat.cols());
            //     mat.block(0, mat.cols() - threshold_mat.cols(), mat.rows(), threshold_mat.cols()) = threshold_mat;
            //     // std::cout << threshold_mat.rows() << "mat " << threshold_mat.cols() <<std::endl;
            }
            // Eigen::MatrixXd result = (mat.rowwise().sum()) / 2;
            // std::cout << result <<std::endl;
        }
    }
}

int main()
{
    // basic information for apriltag
    // int tagRows = 6;
    // int tagCols = 6;
    // double tagSize = 0.088;
    // double tagSpacing = 0.0264;
    // double threshold = 1e-9;

    int tagRows = 6;
    int tagCols = 3;
    double tagSize = 0.205;
    double tagSpacing = 0.3;
    double threshold = 1e-9;

    // set detect single image or kalibr_image?
    bool single = true;
    // bool save_txt = true;
    bool save_result = true;
    //
    if (single)
    {
        std::string image_path = "/data/suween/corner_detect/data/image.jpg";
        single_detection(image_path, tagRows, tagCols, tagSize, tagSpacing, save_result);
    }
    else
    {
        std::string datapath = "/data/suween/data_collect/videopipe/images1121/first";
        multi_detection(datapath, tagRows, tagCols,
                        tagSize, tagSpacing, save_result);
    }
}
#pragma once

#include <fstream>
#include <opencv2/core.hpp>
#include "jsoncpp/json/json.h"

/**
 * @brief 标定参数
 *
 */
class CalibrationParam
{
private:
    std::string name;   // 名称
    std::string target; // 平移和旋转相对的目标
    cv::Mat extrinsic;
    cv::Mat rotation;    // 旋转矩阵，表示在目标的坐标系下的旋转
    cv::Mat translation; // 平移向量，表示在目标的坐标系下的平移

    std::string channel;  // 在数据集中的通道或rostopic
    std::string modality; // 自身类型

    cv::Size image_size;          // 图像大小 [w,h]，其中w代表图像的宽，h代表图像的高
    cv::Mat intrinsic;            // 内参矩阵 [f/dx,skew,u0,0,f/dy,v0,0,0,1]，其中fx、fy代表使用像素来描述x轴、y轴方向焦距的长度，u0、v0代表光心（相机坐标系原点）在像素坐标系的像素坐标，skew代表扭曲参数
    cv::Mat distortion;           // 畸变参数 [k1,k2,p1,p2[,k3]]，其中k代表径向畸变，p代表切向畸变
    cv::Mat undistort_intrinsic;  // 去畸变后的图像再次标定的内参矩阵 [f/dx,skew,u0,0,f/dy,v0,0,0,1]
    cv::Mat undistort_distortion; // 去畸变后的图像再次标定的畸变参数 [k1,k2,p1,p2[,k3]]，其中k代表径向畸变，p代表切向畸变

    Json::Value loadJson(std::string calibration_param_path);
    Json::Value dumpJson();

    std::string loadString(Json::Value root, std::string key, Json::ArrayIndex size);
    cv::Mat loadMat(Json::Value root, std::string key, Json::ArrayIndex size);
    cv::Size loadSize(Json::Value root, std::string key, Json::ArrayIndex size);

public:
    CalibrationParam();

    CalibrationParam(const CalibrationParam &calibration_param);

    /**
     * @brief 读取标定参数json文件构造标定参数
     *
     * @param calibration_param_path 标定参数文件路径
     */
    CalibrationParam(std::string calibration_param_path);
    ~CalibrationParam();

    /**
     * @brief 设置标定参数的name
     *
     * @param name 标定参数的name
     */
    void setName(std::string name);
    /**
     * @brief 设置标定参数的target
     *
     * @param target 标定参数的target
     */
    void setTarget(std::string target);
    /**
     * @brief 设置标定参数的extrinsic
     *
     * @param extrinsic 标定参数的extrinsic
     */
    void setExtrinsic(cv::Mat extrinsic);
    /**
     * @brief 设置标定参数的rotation
     *
     * @param rotation 标定参数的rotation
     */
    void setRotation(cv::Mat rotation);
    /**
     * @brief 设置标定参数的translation
     *
     * @param translation 标定参数的translation
     */
    void setTranslation(cv::Mat translation);
    /**
     * @brief 设置标定参数的channel
     *
     * @param channel 标定参数的channel
     */
    void setChannel(std::string channel);
    /**
     * @brief 设置标定参数的modality
     *
     * @param modality 标定参数的modality
     */
    void setModality(std::string modality);
    /**
     * @brief 设置标定参数的image_size
     *
     * @param image_size 标定参数的image_size
     */
    void setImageSize(cv::Size image_size);
    /**
     * @brief 设置标定参数的intrinsic
     *
     * @param intrinsic 标定参数的intrinsic
     */
    void setIntrinsic(cv::Mat intrinsic);
    /**
     * @brief 设置标定参数的distortion
     *
     * @param distortion 标定参数的distortion
     */
    void setDistortion(cv::Mat distortion);
    /**
     * @brief 设置标定参数的undistort_intrinsic
     *
     * @param undistort_intrinsic 标定参数的undistort_intrinsic
     */
    void setUndistortIntrinsic(cv::Mat undistort_intrinsic);
    /**
     * @brief 设置标定参数的undistort_distortion
     *
     * @param undistort_distortion 标定参数的undistort_distortion
     */
    void setUndistortDistortion(cv::Mat undistort_distortion);

    /**
     * @brief 读取标定参数json文件更新标定参数的name
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadName(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的target
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadTarget(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的extrinsic
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadExtrinsic(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的rotation
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadRotation(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的translation
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadTranslation(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的channel
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadChannel(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的modality
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadModality(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的image_size
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadImageSize(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的intrinsic
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadIntrinsic(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的distortion
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadDistortion(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的undistort_intrinsic
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadUndistortIntrinsic(std::string calibration_param_path);
    /**
     * @brief 读取标定参数json文件更新标定参数的undistort_distortion
     *
     * @param calibration_param_path 标定参数文件路径
     */
    void loadUndistortDistortion(std::string calibration_param_path);

    /**
     * @brief 获取标定参数的name
     *
     * @return std::string 标定参数的name
     */
    std::string getName() const;
    /**
     * @brief 获取标定参数的target
     *
     * @return std::string 标定参数的target
     */
    std::string getTarget() const;
    /**
     * @brief 获取标定参数的extrinsic
     *
     * @return cv::Mat 标定参数的extrinsic
     */
    cv::Mat getExtrinsic() const;
    /**
     * @brief 获取标定参数的rotation
     *
     * @return cv::Mat 标定参数的rotation
     */
    cv::Mat getRotation() const;
    /**
     * @brief 获取标定参数的translation
     *
     * @return cv::Mat 标定参数的translation
     */
    cv::Mat getTranslation() const;

    /**
     * @brief 获取标定参数的channel
     *
     * @return std::string 标定参数的channel
     */
    std::string getChannel() const;
    /**
     * @brief 获取标定参数的modality
     *
     * @return std::string 标定参数的modality
     */
    std::string getModality() const;
    /**
     * @brief 获取标定参数的image_size
     *
     * @return cv::Size 标定参数的image_size
     */
    cv::Size getImageSize() const;
    /**
     * @brief 获取标定参数的intrinsic
     *
     * @return cv::Mat 标定参数的intrinsic
     */
    cv::Mat getIntrinsic() const;
    /**
     * @brief 获取标定参数的distortion
     *
     * @return cv::Mat 标定参数的distortion
     */
    cv::Mat getDistortion() const;
    /**
     * @brief 获取标定参数的undistort_intrinsic
     *
     * @return cv::Mat 标定参数的undistort_intrinsic
     */
    cv::Mat getUndistortIntrinsic() const;
    /**
     * @brief 获取标定参数的undistort_distortion
     *
     * @return cv::Mat 标定参数的undistort_distortion
     */
    cv::Mat getUndistortDistortion() const;

    void save(std::string calibration_param_path);

    friend std::ostream &operator<<(std::ostream &os, const CalibrationParam &calibration_param);
};
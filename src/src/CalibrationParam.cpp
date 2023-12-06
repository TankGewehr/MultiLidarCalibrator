#include "CalibrationParam.h"

Json::Value CalibrationParam::loadJson(std::string calibration_param_path)
{
    Json::Reader reader;
    Json::Value root;
    std::ifstream is(calibration_param_path, std::ios::binary);
    if (!is.is_open())
        throw std::runtime_error("Error opening " + calibration_param_path);
    if (!reader.parse(is, root))
        throw std::invalid_argument("Error json file format " + calibration_param_path);
    is.close();
    return root;
}

Json::Value CalibrationParam::dumpJson()
{
    Json::Value root;
    Json::Value target = this->target;
    Json::Value rotation(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->rotation.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->rotation.cols; col++)
        {
            rotation.append(this->rotation.at<double>(row, col));
        }
    }
    Json::Value translation(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->translation.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->translation.cols; col++)
        {
            translation.append(this->translation.at<double>(row, col));
        }
    }
    Json::Value channel = this->channel;
    Json::Value modality = this->modality;
    Json::Value image_size(Json::arrayValue);
    image_size.append(this->image_size.width);
    image_size.append(this->image_size.height);
    Json::Value intrinsic(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->intrinsic.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->intrinsic.cols; col++)
        {
            intrinsic.append(this->intrinsic.at<double>(row, col));
        }
    }
    Json::Value distortion(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->distortion.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->distortion.cols; col++)
        {
            distortion.append(this->distortion.at<double>(row, col));
        }
    }
    Json::Value undistort_intrinsic(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->undistort_intrinsic.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->undistort_intrinsic.cols; col++)
        {
            undistort_intrinsic.append(this->undistort_intrinsic.at<double>(row, col));
        }
    }
    Json::Value undistort_distortion(Json::arrayValue);
    for (Json::ArrayIndex row = 0; row < this->undistort_distortion.rows; row++)
    {
        for (Json::ArrayIndex col = 0; col < this->undistort_distortion.cols; col++)
        {
            undistort_distortion.append(this->undistort_distortion.at<double>(row, col));
        }
    }

    root["target"] = target;
    root["rotation"] = rotation;
    root["translation"] = translation;
    root["channel"] = channel;
    root["modality"] = modality;
    root["image_size"] = image_size;
    root["intrinsic"] = intrinsic;
    root["distortion"] = distortion;
    root["undistort_intrinsic"] = undistort_intrinsic;
    root["undistort_distortion"] = undistort_distortion;
    return root;
}

std::string CalibrationParam::loadString(Json::Value root, std::string key, Json::ArrayIndex size)
{
    if (root[key].isNull() || root[key].type() != Json::stringValue)
        throw std::invalid_argument("Error " + key + " type");
    if (size && root[key].size() != size)
        throw std::length_error("Error " + key + " size");
    std::string data = root[key].asCString();
    return data;
}

cv::Mat CalibrationParam::loadMat(Json::Value root, std::string key, Json::ArrayIndex size)
{
    if (root[key].isNull() || root[key].type() != Json::arrayValue)
        throw std::invalid_argument("Error " + key + " type");
    if (size && root[key].size() != size)
        throw std::length_error("Error " + key + " size");
    std::vector<double> data;
    for (Json::ArrayIndex index = 0; index < root[key].size(); index++)
        data.emplace_back(root[key][index].asDouble());
    return cv::Mat(data).clone();
}

cv::Size CalibrationParam::loadSize(Json::Value root, std::string key, Json::ArrayIndex size)
{
    if (root[key].isNull() || root[key].type() != Json::arrayValue)
        throw std::invalid_argument("Error " + key + " type");
    if (size && root[key].size() != size)
        throw std::length_error("Error " + key + " size");
    cv::Size data(root[key][0].asInt(), root[key][1].asInt());
    return data;
}

CalibrationParam::CalibrationParam()
{
    this->setName("");
    this->setTarget("");
    this->setExtrinsic(cv::Mat::eye(cv::Size(4, 4), CV_64FC1));
    this->rotation = this->extrinsic(cv::Range(0, 3), cv::Range(0, 3));
    this->translation = this->extrinsic(cv::Range(0, 3), cv::Range(3, 4));

    this->setChannel("");
    this->setModality("");
}

CalibrationParam::CalibrationParam(const CalibrationParam &calibration_param)
{
    this->setName(calibration_param.getName());
    this->setTarget(calibration_param.getTarget());
    this->setExtrinsic(calibration_param.getExtrinsic());
    this->rotation = this->extrinsic(cv::Range(0, 3), cv::Range(0, 3));
    this->translation = this->extrinsic(cv::Range(0, 3), cv::Range(3, 4));

    this->setChannel(calibration_param.getChannel());
    this->setModality(calibration_param.getModality());

    if (this->modality == "camera")
    {
        this->setImageSize(calibration_param.getImageSize());
        this->setIntrinsic(calibration_param.getIntrinsic());
        this->setDistortion(calibration_param.getDistortion());
        this->setUndistortIntrinsic(calibration_param.getUndistortIntrinsic());
        this->setUndistortDistortion(calibration_param.getUndistortDistortion());
    }
}

CalibrationParam::CalibrationParam(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);

    this->setName(calibration_param_path.substr(calibration_param_path.find_last_of("/\\") + 1, calibration_param_path.find_last_of('.') - calibration_param_path.find_last_of("/\\") - 1));
    this->setTarget(this->loadString(root, "target", 0));
    this->setExtrinsic(cv::Mat::eye(cv::Size(4, 4), CV_64FC1));
    this->rotation = this->extrinsic(cv::Range(0, 3), cv::Range(0, 3));
    this->translation = this->extrinsic(cv::Range(0, 3), cv::Range(3, 4));
    this->setRotation(this->loadMat(root, "rotation", 9));
    this->setTranslation(this->loadMat(root, "translation", 3));

    this->setChannel(this->loadString(root, "channel", 0));
    this->setModality(this->loadString(root, "modality", 0));

    if (this->modality == "camera")
    {
        this->setImageSize(this->loadSize(root, "image_size", 2));
        this->setIntrinsic(this->loadMat(root, "intrinsic", 9));
        this->setDistortion(this->loadMat(root, "distortion", 0));
        this->setUndistortIntrinsic(this->loadMat(root, "undistort_intrinsic", 9));
        this->setUndistortDistortion(this->loadMat(root, "undistort_distortion", 0));
    }
}

CalibrationParam::~CalibrationParam()
{
}

void CalibrationParam::setName(std::string name)
{
    this->name = name;
}

void CalibrationParam::setTarget(std::string target)
{
    this->target = target;
}

void CalibrationParam::setExtrinsic(cv::Mat extrinsic)
{
    if (((extrinsic.rows != 1 && extrinsic.cols != 16) || (extrinsic.rows != 4 && extrinsic.cols != 4)) && extrinsic.type() != CV_64FC1)
        throw std::invalid_argument("extrinsic take 1*16 or 4*4 CV_64FC1");
    extrinsic.reshape(1, 4).copyTo(this->extrinsic);
}

void CalibrationParam::setRotation(cv::Mat rotation)
{
    if (((rotation.rows != 1 && rotation.cols != 9) || (rotation.rows != 3 && rotation.cols != 3)) && rotation.type() != CV_64FC1)
        throw std::invalid_argument("rotation take 1*9 or 3*3 CV_64FC1");
    rotation.reshape(1, 3).copyTo(this->rotation);
}

void CalibrationParam::setTranslation(cv::Mat translation)
{
    if (((translation.rows != 1 && translation.cols != 3) || (translation.rows != 3 && translation.cols != 1)) && translation.type() != CV_64FC1)
        throw std::invalid_argument("translation take 1*3 or 3*1 CV_64FC1");
    translation.reshape(1, 3).copyTo(this->translation);
}

void CalibrationParam::setChannel(std::string channel)
{
    this->channel = channel;
}

void CalibrationParam::setModality(std::string modality)
{
    this->modality = modality;
}

void CalibrationParam::setImageSize(cv::Size image_size)
{
    this->image_size = image_size;
}

void CalibrationParam::setIntrinsic(cv::Mat intrinsic)
{
    if (((intrinsic.rows != 1 && intrinsic.cols != 9) || (intrinsic.rows != 3 && intrinsic.cols != 3)) && intrinsic.type() != CV_64FC1)
        throw std::invalid_argument("intrinsic take 1*9 or 3*3 CV_64FC1");
    this->intrinsic = intrinsic.reshape(1, 3).clone();
}

void CalibrationParam::setDistortion(cv::Mat distortion)
{
    this->distortion = distortion.reshape(1, 1).clone();
}

void CalibrationParam::setUndistortIntrinsic(cv::Mat undistort_intrinsic)
{
    if (((undistort_intrinsic.rows != 1 && undistort_intrinsic.cols != 9) || (undistort_intrinsic.rows != 3 && undistort_intrinsic.cols != 3)) && undistort_intrinsic.type() != CV_64FC1)
        throw std::invalid_argument("undistort_intrinsic take 1*9 or 3*3 CV_64FC1");
    this->undistort_intrinsic = undistort_intrinsic.reshape(1, 3).clone();
}

void CalibrationParam::setUndistortDistortion(cv::Mat undistort_distortion)
{
    this->undistort_distortion = undistort_distortion.reshape(1, 1).clone();
}

void CalibrationParam::loadName(std::string calibration_param_path)
{
    this->setName(calibration_param_path.substr(calibration_param_path.find_last_of("/\\") + 1, calibration_param_path.find_last_of('.') - calibration_param_path.find_last_of("/\\") - 1));
}

void CalibrationParam::loadTarget(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setTarget(this->loadString(root, "target", 0));
}

void CalibrationParam::loadExtrinsic(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setRotation(this->loadMat(root, "rotation", 9));
    this->setTranslation(this->loadMat(root, "translation", 3));
}

void CalibrationParam::loadRotation(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setRotation(this->loadMat(root, "rotation", 9));
}

void CalibrationParam::loadTranslation(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setTranslation(this->loadMat(root, "translation", 3));
}

void CalibrationParam::loadChannel(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setChannel(this->loadString(root, "channel", 0));
}

void CalibrationParam::loadModality(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setModality(this->loadString(root, "modality", 0));
}

void CalibrationParam::loadImageSize(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setImageSize(this->loadSize(root, "image_size", 2));
}

void CalibrationParam::loadIntrinsic(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setIntrinsic(this->loadMat(root, "intrinsic", 9));
}

void CalibrationParam::loadDistortion(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setDistortion(this->loadMat(root, "distortion", 0));
}

void CalibrationParam::loadUndistortIntrinsic(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setUndistortIntrinsic(this->loadMat(root, "undistort_intrinsic", 9));
}

void CalibrationParam::loadUndistortDistortion(std::string calibration_param_path)
{
    Json::Value root = this->loadJson(calibration_param_path);
    this->setUndistortDistortion(this->loadMat(root, "undistort_distortion", 0));
}

std::string CalibrationParam::getName() const
{
    return this->name;
}

std::string CalibrationParam::getTarget() const
{
    return this->target;
}

cv::Mat CalibrationParam::getExtrinsic() const
{
    return this->extrinsic.clone();
}

cv::Mat CalibrationParam::getRotation() const
{
    return this->rotation.clone();
}

cv::Mat CalibrationParam::getTranslation() const
{
    return this->translation.clone();
}

std::string CalibrationParam::getChannel() const
{
    return this->channel;
}

std::string CalibrationParam::getModality() const
{
    return this->modality;
}

cv::Size CalibrationParam::getImageSize() const
{
    return this->image_size;
}

cv::Mat CalibrationParam::getIntrinsic() const
{
    return this->intrinsic.clone();
}

cv::Mat CalibrationParam::getDistortion() const
{
    return this->distortion.clone();
}

cv::Mat CalibrationParam::getUndistortIntrinsic() const
{
    return this->undistort_intrinsic.clone();
}

cv::Mat CalibrationParam::getUndistortDistortion() const
{
    return this->undistort_distortion.clone();
}

void CalibrationParam::save(std::string calibration_param_path)
{
    std::ofstream os;
    os.open(calibration_param_path, std::ios::out);
    if (!os.is_open())
        throw std::runtime_error("Error opening " + calibration_param_path);
    Json::StyledWriter sw;
    os << sw.write(dumpJson()) << std::flush;
    os.close();
}

std::ostream &operator<<(std::ostream &os, const CalibrationParam &calibration_param)
{
    os << "name:\n"
       << calibration_param.name << "\n\n"
       << "target:\n"
       << calibration_param.target << "\n\n"
       << "extrinsic:\n"
       << calibration_param.extrinsic << "\n\n"
       << "rotation:\n"
       << calibration_param.rotation << "\n\n"
       << "translation:\n"
       << calibration_param.translation << "\n\n"
       << "channel:\n"
       << calibration_param.channel << "\n\n"
       << "modality:\n"
       << calibration_param.modality << "\n\n"
       << "image_size:\n"
       << calibration_param.image_size << "\n\n"
       << "intrinsic:\n"
       << calibration_param.intrinsic << "\n\n"
       << "distortion:\n"
       << calibration_param.distortion << "\n\n"
       << "undistort_intrinsic:\n"
       << calibration_param.undistort_intrinsic << "\n\n"
       << "undistort_distortion:\n"
       << calibration_param.undistort_distortion;
    return os;
}
#include <iostream>
#include "../include/Camera.h"
#include "../include/ScaramuzzaCamera.h"

#include <opencv2/calib3d/calib3d.hpp>

Camera::Parameters::Parameters(ModelType modelType)
    : m_modelType(modelType), m_imageWidth(0), m_imageHeight(0)
{
  switch (modelType)
  {
  case KANNALA_BRANDT:
    m_nIntrinsics = 8;
    break;
  case PINHOLE:
    m_nIntrinsics = 8;
    break;
  case SCARAMUZZA:
    m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
    break;
  case MEI:
  default:
    m_nIntrinsics = 9;
  }
}

Camera::Parameters::Parameters(ModelType modelType,
                               const std::string &cameraName, int w, int h)
    : m_modelType(modelType),
      m_cameraName(cameraName),
      m_imageWidth(w),
      m_imageHeight(h)
{
  switch (modelType)
  {
  case KANNALA_BRANDT:
    m_nIntrinsics = 8;
    break;
  case PINHOLE:
    m_nIntrinsics = 8;
    break;
  case SCARAMUZZA:
    m_nIntrinsics = SCARAMUZZA_CAMERA_NUM_PARAMS;
    break;
  case MEI:
  default:
    m_nIntrinsics = 9;
  }
}

Camera::ModelType &Camera::Parameters::modelType(void) { return m_modelType; }

std::string &Camera::Parameters::cameraName(void) { return m_cameraName; }

int &Camera::Parameters::imageWidth(void) { return m_imageWidth; }

int &Camera::Parameters::imageHeight(void) { return m_imageHeight; }

Camera::ModelType Camera::Parameters::modelType(void) const
{
  return m_modelType;
}

const std::string &Camera::Parameters::cameraName(void) const
{
  return m_cameraName;
}

int Camera::Parameters::imageWidth(void) const { return m_imageWidth; }

int Camera::Parameters::imageHeight(void) const { return m_imageHeight; }

int Camera::Parameters::nIntrinsics(void) const { return m_nIntrinsics; }

cv::Mat &Camera::mask(void) { return m_mask; }

const cv::Mat &Camera::mask(void) const { return m_mask; }

void Camera::estimateExtrinsics(const std::vector<cv::Point3f> &objectPoints,
                                const std::vector<cv::Point2f> &imagePoints,
                                cv::Mat &rvec, cv::Mat &tvec) const
{
  std::vector<cv::Point2f> Ms(imagePoints.size());
  for (size_t i = 0; i < Ms.size(); ++i)
  {
    Eigen::Vector3d P;
    liftProjective(Eigen::Vector2d(imagePoints.at(i).x, imagePoints.at(i).y),
                   P);

    P /= P(2);

    Ms.at(i).x = P(0);
    Ms.at(i).y = P(1);
  }

  // assume unit focal length, zero principal point, and zero distortion
  cv::solvePnP(objectPoints, Ms, cv::Mat::eye(3, 3, CV_64F), cv::noArray(),
               rvec, tvec);
}

double Camera::reprojectionDist(const Eigen::Vector3d &P1,
                                const Eigen::Vector3d &P2) const
{
  Eigen::Vector2d p1, p2;

  spaceToPlane(P1, p1);
  spaceToPlane(P2, p2);

  return (p1 - p2).norm();
}

double Camera::reprojectionError(
    const std::vector<std::vector<cv::Point3f>> &objectPoints,
    const std::vector<std::vector<cv::Point2f>> &imagePoints,
    const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
    cv::OutputArray _perViewErrors) const
{
  int imageCount = objectPoints.size();
  size_t pointsSoFar = 0;
  double totalErr = 0.0;

  bool computePerViewErrors = _perViewErrors.needed();
  cv::Mat perViewErrors;
  if (computePerViewErrors)
  {
    _perViewErrors.create(imageCount, 1, CV_64F);
    perViewErrors = _perViewErrors.getMat();
  }

  for (int i = 0; i < imageCount; ++i)
  {
    size_t pointCount = imagePoints.at(i).size();

    pointsSoFar += pointCount;

    std::vector<cv::Point2f> estImagePoints;
    projectPoints(objectPoints.at(i), rvecs.at(i), tvecs.at(i), estImagePoints);

    double err = 0.0;
    for (size_t j = 0; j < imagePoints.at(i).size(); ++j)
    {
      err += cv::norm(imagePoints.at(i).at(j) - estImagePoints.at(j));
    }

    if (computePerViewErrors)
    {
      perViewErrors.at<double>(i) = err / pointCount;
    }

    totalErr += err;
  }

  return totalErr / pointsSoFar;
}

double Camera::reprojectionError(const Eigen::Vector3d &P,
                                 const Eigen::Quaterniond &camera_q,
                                 const Eigen::Vector3d &camera_t,
                                 const Eigen::Vector2d &observed_p) const
{
  Eigen::Vector3d P_cam = camera_q.toRotationMatrix() * P + camera_t;

  Eigen::Vector2d p;
  spaceToPlane(P_cam, p);

  return (p - observed_p).norm();
}

void Camera::projectPoints(const std::vector<cv::Point3f> &objectPoints,
                           const cv::Mat &rvec, const cv::Mat &tvec,
                           std::vector<cv::Point2f> &imagePoints) const
{
  // project 3D object points to the image plane
  imagePoints.reserve(objectPoints.size());

  // double
  cv::Mat R0;
  cv::Rodrigues(rvec, R0);

  
  // std::cout<<"R0:"<<R0<<std::endl;
  // Rodrigues变换  int cvRodrigues2( const CvMat* src, CvMat* dst, CvMat* jacobian=0 );
  //   src为输入的旋转向量（3x1或者1x3）或者旋转矩阵（3x3）。dst为输出的旋转矩阵（3x3）或者旋转向量（3x1或者1x3）。jacobian为可选的输出雅可比矩阵（3x9或者9x3），是输入与输出数组的偏导数。

  Eigen::MatrixXd R(3, 3);
  R << R0.at<double>(0, 0), R0.at<double>(0, 1), R0.at<double>(0, 2),
      R0.at<double>(1, 0), R0.at<double>(1, 1), R0.at<double>(1, 2),
      R0.at<double>(2, 0), R0.at<double>(2, 1), R0.at<double>(2, 2);

  std::cout<<"R:"<<R<<std::endl;

  Eigen::Vector3d t;
  t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  std::cout<<"t:"<<t<<std::endl;

// std::cout<<"objectPoints 's size : "<<objectPoints.size()<<std::endl;

  for (size_t i = 0; i < objectPoints.size(); ++i)
  {

    const cv::Point3f &objectPoint = objectPoints.at(i);

    // Rotate and translate
    Eigen::Vector3d P;
    P << objectPoint.x, objectPoint.y, objectPoint.z;

    P = R * P + t;  //R is  identity matrix (3x3)   t is zero matrix(3x1)    therefore the result of the equation is P itself.



    Eigen::Vector2d p;
    spaceToPlane(P, p);
    imagePoints.push_back(cv::Point2f(p(0), p(1)));
    // std::cout<<cv::Point2f(p(0), p(1))<<std::endl;
  }
}

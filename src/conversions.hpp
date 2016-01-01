#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <opencv2/core/eigen.hpp>

#include <fstream>
namespace image_pipeline
{
namespace impl
{
template<typename PointT, typename DepthT>
inline void
cvToCloudOrganized(const cv::Mat_<cv::Vec<DepthT, 3> >& points3d, pcl::PointCloud<PointT>& cloud)
{
  typedef cv::Vec<DepthT, 3> DepthVec;
  assert(points3d.channels() == 3);

  int width = points3d.cols, height = points3d.rows;

  cloud.points.resize(width * height);
  cloud.width = width;
  cloud.height = height;

  for (int v = 0; v < height; ++v)
  {
    const DepthVec* begin = reinterpret_cast<const DepthVec*>(points3d.ptr(v));
    for (int u = 0; u < width; ++u, ++begin)
    {
      PointT& p = cloud(u, v);
      p.x = (*begin)[0];
      p.y = (*begin)[1];
      p.z = (*begin)[2];
    }
  }
}

template<typename PointT, typename DepthT>
inline void
cvToCloudRGBOrganized(const cv::Mat_<cv::Vec<DepthT, 3> >& points3d, const cv::Mat& rgb, pcl::PointCloud<PointT>& cloud)
{
  typedef cv::Vec<DepthT, 3> DepthVec;
  assert(points3d.channels() == 3);
  assert(rgb.channels() == 3);
  assert(rgb.depth() == 0);

  assert(points3d.cols == rgb.cols && points3d.rows == rgb.rows);

  int width = points3d.cols, height = points3d.rows;

  cloud.points.resize(width * height);
  cloud.width = width;
  cloud.height = height;

  for (int v = 0; v < height; ++v)
  {
    const DepthVec* begin = reinterpret_cast<const DepthVec*>(points3d.ptr(v));
    const cv::Vec3b* begin_rgb = rgb.ptr<cv::Vec3b>(v);
    for (int u = 0; u < width; ++u, ++begin_rgb, ++begin)
    {
      PointT& p = cloud(u, v);
      p.x = (*begin)[0];
      p.y = (*begin)[1];
      p.z = (*begin)[2];
      // bgr layout
      p.r = (*begin_rgb)[0];
      p.g = (*begin_rgb)[1];
      p.b = (*begin_rgb)[2];
    }
  }
}
}

  /**
   * \breif convert an opencv collection of points to a pcl::PoinCloud, your opencv mat should have NAN's for invalid points.
   * @param points3d opencv matrix of nx1 3 channel points
   * @param cloud output cloud
   * @param rgb the rgb, required, will color points
   * @param mask the mask, required, must be same size as rgb
   */
  inline void
  cvToCloudXYZRGB(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<pcl::PointXYZRGB>& cloud, const cv::Mat& rgb,
                  const cv::Mat& mask, bool brg = true)
  {
    cloud.clear();
    cv::Mat_<cv::Point3f>::const_iterator point_it = points3d.begin(), point_end = points3d.end();
    cv::Mat_<cv::Vec3b>::const_iterator rgb_it = rgb.begin<cv::Vec3b>();
    cv::Mat_<uchar>::const_iterator mask_it;
    if(!mask.empty())
      mask_it = mask.begin<uchar>();
    for (; point_it != point_end; ++point_it, ++rgb_it)
    {
      if(!mask.empty())
      {
        ++mask_it;
        if (!*mask_it)
          continue;
      }

      cv::Point3f p = *point_it;
      if (p.x != p.x || p.y != p.y || p.z != p.z) //throw out NANs
        continue;
      pcl::PointXYZRGB cp;
      cp.x = p.x;
      cp.y = p.y;
      cp.z = p.z;
      cp.r = (*rgb_it)[2]; //expecting in BGR format.
      cp.g = (*rgb_it)[1];
      cp.b = (*rgb_it)[0];
      cloud.push_back(cp);
    }
  }

  template<typename PointT>
  inline void
  cvToCloud(const cv::Mat_<cv::Point3f>& points3d, pcl::PointCloud<PointT>& cloud, const cv::Mat& mask = cv::Mat())
  {
    cloud.clear();
    cv::Mat_<cv::Point3f>::const_iterator point_it = points3d.begin(), point_end = points3d.end();
    const bool has_mask = !mask.empty();
    cv::Mat_<uchar>::const_iterator mask_it;
    if (has_mask)
      mask_it = mask.begin<uchar>();
    cloud.reserve(cloud.width*cloud.height);
    for (; point_it != point_end; ++point_it, (has_mask ? ++mask_it : mask_it))
    {
      if (has_mask && !*mask_it)
        continue;
      cv::Point3f p = *point_it;
      if (p.x != p.x || p.y != p.y || p.z != p.z) //throw out NANs
        continue;
      PointT cp;
      cp.x = p.x;
      cp.y = p.y;
      cp.z = p.z;
      cloud.push_back(cp);
    }
  }

  template<typename PointT>
  inline void
  cvToCloudOrganized(const cv::Mat& points3d, pcl::PointCloud<PointT>& cloud)
  {
    assert(points3d.channels() == 3);
    assert(points3d.depth() == 5 || points3d.depth() == 6);

    if (points3d.depth() == 5)
      impl::cvToCloudOrganized<PointT, float>(points3d, cloud);
    else
      impl::cvToCloudOrganized<PointT, double>(points3d, cloud);
  }

  template<typename PointT>
  inline void
  cvToCloudRGBOrganized(const cv::Mat& points3d, const cv::Mat& rgb, pcl::PointCloud<PointT>& cloud)
  {
    assert(points3d.channels() == 3);
    assert(rgb.channels() == 3);
    assert(rgb.depth() == 0);

    assert(points3d.cols == rgb.cols && points3d.rows == rgb.rows);

    assert(points3d.depth() == 5 || points3d.depth() == 6);

    if (points3d.depth() == 5)
      impl::cvToCloudRGBOrganized<PointT, float>(points3d, rgb, cloud);
    else
      impl::cvToCloudRGBOrganized<PointT, double>(points3d, rgb, cloud);
  }

      /**
       * p = R*x + T
       * if inverse:
       *  x = R^1*(p - T)
       */
  inline Eigen::Affine3f
  RT2Transform(cv::Mat& R, cv::Mat& T, bool inverse)
  {
    //convert the tranform from our fiducial markers to
    //the Eigen
    Eigen::Matrix<float, 3, 3> eR;
    Eigen::Vector3f eT;
    cv::cv2eigen(R, eR);
    cv::cv2eigen(T, eT);
    // p = R*x + T
    Eigen::Affine3f transform;
    if (inverse)
    {
      //x = R^1*(p - T)
      transform = Eigen::Translation3f(-eT);
      transform.prerotate(eR.transpose());
    }
    else
    {
      //p = R*x + T
      transform = Eigen::AngleAxisf(eR);
      transform.translate(eT);
    }
    return transform;
  }
}

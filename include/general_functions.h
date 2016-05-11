#include <headers.h>
#include <time.h>


void bin_into_histogram(float temp1, float histogram[], int& histogram_samples)
{
    if ( temp1 >= 0 and temp1 < 10)
    {
        histogram[0]++;
        histogram_samples++;
    }
    else if ( temp1 >= 10 and temp1 < 20)
    {
        histogram[1]++;
        histogram_samples++;
    }
    else if ( temp1 >= 20 and temp1 < 30)
    {
        histogram[2]++;
        histogram_samples++;
    }
    else if ( temp1 >= 30 and temp1 < 40)
    {
        histogram[3]++;
        histogram_samples++;
    }
    else if ( temp1 >= 40 and temp1 < 50)
    {
        histogram[4]++;
        histogram_samples++;
    }
    else if ( temp1 >= 50 and temp1 < 60)
    {
        histogram[5]++;
        histogram_samples++;
    }
    else if ( temp1 >= 60 and temp1 < 70)
    {
        histogram[6]++;
        histogram_samples++;
    }
    else if ( temp1 >= 70 and temp1 < 80)
    {
        histogram[7]++;
        histogram_samples++;
    }
    else if ( temp1 >= 80 and temp1 < 90)
    {
        histogram[8]++;
        histogram_samples++;
    }
    else if ( temp1 >= 90 and temp1 < 100)
    {
        histogram[9]++;
        histogram_samples++;
    }
    else if ( temp1 >= 100 and temp1 < 110)
    {
        histogram[10]++;
        histogram_samples++;
    }
    else if ( temp1 >= 110 and temp1 < 120)
    {
        histogram[11]++;
        histogram_samples++;
    }
    else if ( temp1 >= 120 and temp1 < 130)
    {
        histogram[12]++;
        histogram_samples++;
    }
    else if ( temp1 >= 130 and temp1 < 140)
    {
        histogram[13]++;
        histogram_samples++;
    }
    else if ( temp1 >= 140 and temp1 < 150)
    {
        histogram[14]++;
        histogram_samples++;
    }
    else if ( temp1 >= 150 and temp1 < 160)
    {
        histogram[15]++;
        histogram_samples++;
    }
    else if ( temp1 >= 160 and temp1 < 170)
    {
        histogram[16]++;
        histogram_samples++;
    }
    else if ( temp1 >= 170 and temp1 < 180)
    {
        histogram[17]++;
        histogram_samples++;
    }
    else
    {
        std::cout << "\n\n\nDiHedral angle is out of BIN RANGE in histogram ! ! !" << std::endl;
        std::cout << "And the value is :  " << temp1 << "\n\n\n" << std::endl;
    }
}

void normalize_the_histogram (float histogram[], int& histogram_samples)
{
    for ( int i = 0; i < 18; ++i)
    {
        histogram[i] = histogram[i]/histogram_samples;
    }
}

void calculate_skewness_and_kurtosis(double &skewness, double &kurtosis, float histogram[], std::vector<double>& hist_vector)
{
    for (size_t i = 0; i < 18; i++)
    {
        double x = histogram[i];
        hist_vector.push_back(x);
    }
    int n = 18;

    sort(hist_vector.begin(), hist_vector.end());

    //    for (size_t i = 0; i < n; i++)
    //        cout << v[i] << " ";

    double sum = accumulate(hist_vector.begin(), hist_vector.end(), 0);
    double avg = (double)sum/n;
    double var = 0;
    double k = 0;
    skewness = kurtosis = 0;

    for (size_t i = 0; i < n; i++)
    {
        var += (hist_vector[i] - avg)*(hist_vector[i] - avg);
    }
    var = (double)(var)/(n);

    //cout << "SUM = " << sum << endl;
    //cout << "AVG = " << avg << endl;
    //cout << "VAR = " << var << endl;

    double S = (double)sqrt(var);

    for (size_t i = 0; i < n; ++i)
        skewness += (hist_vector[i] - avg)*(hist_vector[i] - avg)*(hist_vector[i] - avg);

    skewness = skewness/(n * S * S * S);

    //cout << "Skewness = " << skewness << endl;

    for (size_t i = 0; i < n; i++)
        k += (hist_vector[i] - avg)*(hist_vector[i] - avg)*(hist_vector[i] - avg)*(hist_vector[i] - avg);

    k = k/(n*S*S*S*S);
    k -= 3;
    kurtosis = k;
    //cout << "Kurtosis = " << k << endl;
}


void calculate_covariance_matrix(Eigen::Matrix3f& covariance_matrix,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud,
                                 std::vector<int>& pointIdxRadiusSearch)
{
    Eigen::Vector4f xyz_centroid;

    pcl::compute3DCentroid (cloud, pointIdxRadiusSearch, xyz_centroid);
    pcl::computeCovarianceMatrix (cloud, pointIdxRadiusSearch, xyz_centroid, covariance_matrix);

}


//! Generates a cube of 1 m edge length in the pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.clear();
  cloud->points.reserve(300000);

  float start = -0.5f;
  float stop = 0.5f;
  float stepSize = 0.005f;

  for (float x=start; x<=stop; x+=stepSize) {
    for (float y=start; y<=stop; y+=stepSize) {
      pcl::PointXYZ point;
      point.x = x + 0.0001f;
      point.y = y + 0.0001f;
      point.z = start + 0.0001f;
   //   point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));

      cloud->points.push_back(point);
      point.x = x + 0.0001f;
      point.y = y + 0.0001f;
      point.z = stop + 0.0001f;
      //point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));

      cloud->points.push_back(point);
    }
  }
  for (float y=start; y<=stop; y+=stepSize) {
    for (float z=start; z<=stop; z+=stepSize) {
      pcl::PointXYZ point;
      point.x = start + 0.0001f;
      point.y = y + 0.0001f;
      point.z = z + 0.0001f;
      //point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);

      point.x = stop + 0.0001f;
      point.y = y + 0.0001f;
      point.z = z + 0.0001f;
      //point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);
    }
  }
  for (float x=start; x<=stop; x+=stepSize) {
    for (float z=start; z<=stop; z+=stepSize) {
      pcl::PointXYZ point;
      point.x = x + 0.0001f;
      point.y = start + 0.0001f;
      point.z = z + 0.0001f;
      //point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);

      point.x = x + 0.0001f;
      point.y = stop + 0.0001f;
      point.z = z + 0.0001f;
     // point.rgb = createPCLRGBFromFloat((point.x+stop), (point.y+stop), (point.z+stop));
      cloud->points.push_back(point);
    }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  cloud->header.frame_id = "/openni_rgb_optical_frame";

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Rotate the cube
  Eigen::Affine3f transform(Eigen::Matrix3f(Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) ));
  pcl::transformPointCloud(*cloud, *transformedCloud, transform);

  return transformedCloud;
}

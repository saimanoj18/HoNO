#include <HoNO.h>


int main (int argc, char** argv)
{

    KPD kp;

    if (pcl::io::loadPCDFile("../scene005_0.pcd",kp.cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << kp.cloud.width * kp.cloud.height << " data points from input cloud  " << std::endl;


    kp.normal_estimation_pcl(0.028);
    kp.local_maxima_of_hist_of_normals_orientations(0.028,12);
    kp.remove_boundary_points(0.01);
    kp.non_maxima_suppression_for_hist_of_norm(0.028);

    cout << "No. of Keypoints : " << kp.fifth_keypoints_cloud.size() << endl;
    // The actual detected keypoints are the in the fifth_keypoints_cloud

    //kp.visualization(1,1,0,0,0);// To visualize Salient regions before boundary removal
    //kp.visualization(1,1,0,0,0);// To visualize Salient regions after boundary removal
    kp.visualization(1,0,0,1,0);// To visualize detected keypoints

    cout << "\nEfficient Coding can reduce the computational time drastically!!!\n" << endl;

    return (0);
}


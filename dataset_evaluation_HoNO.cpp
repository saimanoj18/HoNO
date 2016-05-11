#include <dataset_evaluation_HoNO.h>
#include <dirent.h> // for looping over the files in the directory

bool replace(std::string& str, const std::string& from, const std::string& to)
{
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

/*
All the radius parameters, namely, R normal , R HoNO and R d are
set to the same value as the scale used for experimentation. For
boundary extraction, we "approximately" used 4x mesh resolu-
tion. Finally, T h K , the threshold to detect salient regions is
set to 6, 13, 8, 12 and 12 for Kinect, UWA, SpaceTime, Re-
trieval and StanfordViews datasets respectively.
*/

void find_keypoints(pcl::PointCloud<pcl::PointXYZ> cloud, std::string& temp)
{
    KPD kp;
    kp.cloud = cloud;
    std::cout << "Loaded " << kp.cloud.width * kp.cloud.height << " data points from input cloud  " << std::endl;

    // Testing 6mr, 10mr, 14mr, 18mr
    // mr stands for mesh resolution

    /***************************************************************************/
    /***************************************************************************/
    // Kinect dataset
    // Mesh Resolution 0.002

    // 0.002 x 6 mr, 0.002 x 10 mr, 0.002 x 14 mr, 0.002 x 18 mr,
    kp.normal_estimation_pcl(0.012);// parameter to be changed according to the mesh resolution

    // (0.002 x 6 mr, 6),  (0.002 x 10 mr,6),  (0.002 x 14 mr,6),  (0.002 x 18 mr,6)
    kp.local_maxima_of_hist_of_normals_orientations(0.012,6);// parameter to be changed according to the mesh resolution

    //constant parameter across various mr
    kp.remove_boundary_points(0.01);

    // 0.002 x 6 mr, 0.002 x 10 mr, 0.002 x 14 mr, 0.002 x 18 mr,
    kp.non_maxima_suppression_for_hist_of_norm(0.012);// parameter to be changed according to the mesh resolution

    /***************************************************************************/
    /***************************************************************************/



    /***************************************************************************
    ***************************************************************************

    // UWA dataset
    // Mesh Resolution 0.65

        //Similarly as in the above example change the values of normal_estimation_pcl

    kp.normal_estimation_pcl(3.9);// parameter to be changed according to the mesh resolution
    kp.local_maxima_of_hist_of_normals_orientations(3.9,13);//(first param to be changed, second one is constant)
    kp.remove_boundary_points(1.8);//constant
    kp.non_maxima_suppression_for_hist_of_norm(3.9);//// parameter to be changed according to the mesh resolution


    ***************************************************************************
   ***************************************************************************/




    /***************************************************************************
    ***************************************************************************

    // Space Time
    // Mesh Resolution 0.12

    //Similarly as in the above Kinect dataset change the parameter values accordingly with the mesh resolution

    kp.normal_estimation_pcl(2.16);// parameter to be changed according to the mesh resolution
    kp.local_maxima_of_hist_of_normals_orientations(2.16,8);//(first param to be changed, second one is constant)
    kp.remove_boundary_points(0.24);// //constant
    kp.non_maxima_suppression_for_hist_of_norm(2.16);//// parameter to be changed according to the mesh resolution

    ***************************************************************************
    ***************************************************************************/







    /***************************************************************************
    ***************************************************************************

    // Retreival
    //Mesh Resoltuion 0.0015
    //Similarly as in the above Kinect dataset change the parameter values accordingly with the mesh resolution


    kp.normal_estimation_pcl(0.027);// parameter to be changed according to the mesh resolution
    kp.local_maxima_of_hist_of_normals_orientations(0.027,12);//(first param to be changed, second one is constant)
    kp.remove_boundary_points(0.003);//constant
    kp.non_maxima_suppression_for_hist_of_norm(0.027);// parameter to be changed according to the mesh resolution


    ***************************************************************************
    ***************************************************************************/





    /***************************************************************************
    ***************************************************************************

    // Stanford Views
    // Mesh Resolution 0.001
    //Similarly as in the above Kinect dataset change the parameter values accordingly with the mesh resolution


    kp.normal_estimation_pcl(0.006);// parameter to be changed according to the mesh resolution
    kp.local_maxima_of_hist_of_normals_orientations(0.006,12);//(first param to be changed, second one is constant)
    kp.remove_boundary_points(0.002);//constant
    kp.non_maxima_suppression_for_hist_of_norm(0.006);// parameter to be changed according to the mesh resolution

    ***************************************************************************
    ***************************************************************************/



    cout << "Found " << kp.fifth_keypoints_cloud.size() << " Keypoints"<<endl;



    // Dataset Evaluation Routine

// the feature files are written at these locations. Please change them accordingly with the dataset used for evaluation
    std::string filepath = "../feature_files_Kinect";
    //std::string filepath = "../feature_files_UWA";
    //std::string filepath = "../feature_files_SpaceTime";
    //std::string filepath = "../feature_files_Retrieval";
    //std::string filepath = "../feature_files_StanfordViews";





    if(temp.substr(temp.find_last_of(".") + 1) == "pcd")
    {
        replace(temp, ".pcd", ".feat");
    }


    // VERY IMPORTANT parameter to change.....
    // Please /super_R-6_ for 6mr, /super_R-10_ for 10mr, /super_R-14_ for 14mr and /super_R-18_ for 18mr,
    //Fixed scale over various Radius
    std::string filename = filepath + "/super_R-6_" + temp;// 6mr
    //std::string filename = filepath + "/super_R-10_" + temp;// 10mr
    //std::string filename = filepath + "/super_R-14_" + temp;// 14mr
    //std::string filename = filepath + "/super_R-18_" + temp;// 18mr



    std::ofstream file(filename.c_str(),std::ios_base::trunc);

    if (!file.is_open()) {std::cout << "cannot open file !!! " <<std::endl;}

    int score = 1;
    float scale = 1;

    for (size_t i = 0; i < kp.fifth_keypoint_indices.size(); ++i)
    {
        if(i==0){file << kp.fifth_keypoint_indices.size() << std::endl;}

        double x = kp.fifth_keypoints_cloud.points[i].x;
        double y = kp.fifth_keypoints_cloud.points[i].y;
        double z = kp.fifth_keypoints_cloud.points[i].z;

        file << x << "\t" << y << "\t" << z << "\t" << scale << "\t"<< kp.fifth_keypoint_indices[i]  << "\t" << score <<std::endl;
    }

    file.close();

    //kp.visualization(1,1,0,0);
}


int main (int argc, char** argv)
{
    struct dirent *pDirent;
    DIR *pDir;
    /*
    if (argc < 2)
    {
        printf ("Usage: testprog <dirname>\n");
        return 1;
    }
*/


    //Here are the actual pcd_files that are used for evaluation...Change the folder based on the dataset you want to evaluate
    argv[1] = "../five_datasets/pcd_files_kinect/";
    //argv[1] = "../five_datasets/pcd_files_UWA/";
    //argv[1] = "../five_datasets/pcd_files_spacetime/";
    //argv[1] = "../five_datasets/pcd_files_retreival/";
    //argv[1] = "../five_datasets/pcd_files_stanford/";


    pDir = opendir (argv[1]);
    if (pDir == NULL)
    {
        printf ("Cannot open directory '%s'\n", argv[1]);
        return 1;
    }

    while ((pDirent = readdir(pDir)) != NULL)
    {
        if (strcmp(pDirent->d_name,".")!= 1 || strcmp(pDirent->d_name,"..")!= 1)
            continue;
        printf ("    %s   \n", pDirent->d_name);
        std::string temp(pDirent->d_name);
        std::string filename;
        filename = argv[1]+temp;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }

        std::cout << "Loaded the file" << std::endl;

        find_keypoints(*cloud, temp);

    }
    closedir (pDir);

    return (0);
}

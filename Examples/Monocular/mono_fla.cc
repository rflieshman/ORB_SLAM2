/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadImagesFakeTimestamps(const string& strImageFile,
                              const string &strImagePath,
                              vector<string> &vstrImages,
                              vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_file path_to_image_folder" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    // LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);
    LoadImagesFakeTimestamps(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Only process first N images.
    nImages = 4000;

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        cv::Mat Tcw = SLAM.TrackMonocular(im,tframe);

#if 1
        // Write the live pose.
        std::ofstream myfile;
        myfile.open("orb_slam_poses.csv", std::ios_base::app);
        myfile << std::setprecision(15);

        if (!Tcw.empty()) {
          Eigen::Matrix3d R_in_orbworld;
          Eigen::Vector3d trans_in_orbworld;
          for (int ii = 0; ii < 3; ++ii) {
            for (int jj = 0; jj < 3; ++jj) {
              R_in_orbworld(ii, jj) = Tcw.at<float>(ii, jj);
            }
            trans_in_orbworld(ii) = Tcw.at<float>(ii, 3);
          }
          Eigen::Quaterniond q_in_orbworld(R_in_orbworld);

          // Convert to Right-Down-Forward.
          Eigen::Matrix3d R_to_orb(Eigen::Matrix3d::Zero());
          R_to_orb(0, 0) = 1.0;
          R_to_orb(1, 1) = -1;
          R_to_orb(2, 2) = -1;
          Eigen::Quaterniond q_to_orb(R_to_orb);

          Eigen::Quaterniond q_cam_in_world(q_to_orb.inverse() * q_in_orbworld * q_to_orb);
          Eigen::Vector3d t_cam_in_world(q_to_orb.inverse() * trans_in_orbworld);

          // Print in ASL format.
          myfile << tframe <<
              "," << t_cam_in_world(0) <<
              "," << t_cam_in_world(1) <<
              "," << t_cam_in_world(2) <<
              "," << q_cam_in_world.w() <<
              "," << q_cam_in_world.x() <<
              "," << q_cam_in_world.y() <<
              "," << q_cam_in_world.z() << "\n";
        }
        myfile.close();
#endif

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadImagesFakeTimestamps(const string& strImageList,
                              const string &strImagePath,
                              vector<string> &vstrImages,
                              vector<double> &vTimeStamps)
{
  const double time_start = 0.0;
  const double fps = 60.0;

  int idx = 0;
  ifstream img_file(strImageList);
  std::string img_str;
  while (std::getline(img_file, img_str)) {
    double time = time_start + static_cast<double>(idx) / fps;

    stringstream ss;
    ss << strImagePath + "/" + img_str;
    vstrImages.push_back(ss.str());

    vTimeStamps.push_back(time);

    idx++;
  }
}

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

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 8)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    bool disableViewer = false;
    ORB_SLAM2::disableLoopAndReloc = false;

    disableViewer = bool(atoi(argv[6]));
    ORB_SLAM2::disableLoopAndReloc = bool(atoi(argv[7]));

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,!disableViewer);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vector<float> vLoadTimes;
    vTimesTrack.resize(nImages);
    vLoadTimes.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point tstart = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point tstart = std::chrono::monotonic_clock::now();
#endif
    for(int ni=0; ni<nImages; ni++)
    {
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point tl1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point tl1 = std::chrono::monotonic_clock::now();
#endif
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point tl2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point tl2 = std::chrono::monotonic_clock::now();
#endif
        double tload = std::chrono::duration_cast<std::chrono::duration<double> >(tl2 - tl1).count();

        vLoadTimes[ni] = tload;

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackRGBD(imRGB,imD,tframe);

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
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point tend = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point tend = std::chrono::monotonic_clock::now();
#endif
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    // Load time statistics
    sort(vLoadTimes.begin(),vLoadTimes.end());
    float totalloadtime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totalloadtime+=vLoadTimes[ni];
    }
    cout << "-------" << endl << endl;
    cout << "total tracking time: " << totaltime << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "max tracking time: " << vTimesTrack[nImages-1] << endl;
    cout << "loop time: " << (std::chrono::duration_cast<std::chrono::duration<double> >(tend - tstart).count()) << endl;
    cout << "total image load time: " << totalloadtime << endl;
    cout << "median image load time: " << vLoadTimes[nImages/2] << endl;
    cout << "mean image load time: " << totalloadtime/nImages << endl;
    cout << "max image load time: " << vLoadTimes[nImages-1] << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM(string(argv[5]));

    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


#include <string>
#include <iostream>
#include <algorithm>
//#include <opencv\highgui.h>
#include "mapmerge.h"
#include <cstdlib>
//#include <unistd.h>
#include <opencv2\opencv.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


int setenv(const char* name, const char* value, int overwrite)
{
    int errcode = 0;
    if (!overwrite) {
        size_t envsize = 0;
        errcode = getenv_s(&envsize, NULL, 0, name);
        if (errcode || envsize) return errcode;
    }
    return _putenv_s(name, value);
}
int main(int argc, char** argv)
{
  while(true){


  bool verbose = true;
  string outfile = "final.pgm";
  vector<string> infiles;
  float max_distance = 5.;

  // load the images

  //setenv("ROS_MASTER_URI", "http://192.168.0.110:11311", true);
  //system("echo $ROS_MASTER_URI");
  //system("rosrun map_server map_saver -f map110");
  //
  //setenv("ROS_MASTER_URI", "http://192.168.0.107:11311", true);
  //system("echo $ROS_MASTER_URI");
  //system("rosrun map_server map_saver -f map107");


  cv::Mat im = imread("D:\\1.png",1);
  imshow("kk", im);
  cv::waitKey(0);
  infiles.push_back("C:\\Users\\Ali\\Downloads\\Map-Merge-Tool-master\\Map-Merge-Tool-master\\images\\1.png");
  infiles.push_back("C:\\Users\\Ali\\Downloads\\Map-Merge-Tool-master\\Map-Merge-Tool-master\\images\\2.png");
  outfile = "C:\\Users\\Ali\\Downloads\\Map-Merge-Tool-master\\Map-Merge-Tool-master\\images\\output.png";
  vector<Mat> images;
  for (size_t i=0; i<infiles.size(); i++) {
    Mat image = imread(infiles[i].c_str(), 0); // 0=grayscale
    if (!image.data) {
      cerr << "error: image " << infiles[i] << " not loadable." << endl;
      exit(-1);
    }
    images.push_back(image);
  }
  // create the stitched map
  StitchedMap map(images[0],images[1], max_distance);
  // write to outfile if applicable
  if (outfile.size() != 0) {
    imwrite(outfile, map.get_stitch());
  }

  if (outfile.size() == 0 || verbose) { // generate some output
    cout << "rotation: "          << map.rotation << endl
         << "translation (x,y): " << map.transx << ", " << map.transy << endl
         << "matrix: "            << map.H << endl;
  }

  if (verbose) {
    namedWindow("wrap"); imshow("wrap", map.get_stitch()); imwrite("stitch.pgm", map.get_stitch());
    namedWindow("debug"); imshow("debug", map.get_debug()); imwrite("debug.pgm", map.get_debug());
    waitKey(5000);
    // while ((waitKey(0)&255) != 10) // RETURN
    //   ;
  }
  //usleep(10000000);
  }
  return 0;
}

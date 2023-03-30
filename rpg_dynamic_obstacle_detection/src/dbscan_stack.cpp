#include "opencv2/opencv.hpp"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
using namespace cv;
using namespace std;
#include <map>
#include <sstream>

template <class T> inline std::string to_string(const T &t) {
  std::stringstream ss;
  ss << t;
  return ss.str();
}

class DbScan {
public:
  std::map<int, int> labels;
  vector<Rect> &data;
  int C;
  double eps;
  int mnpts;
  double *dp;
// memoization table in case of complex dist functions
#define DP(i, j) dp[(data.size() * i) + j]
  DbScan(vector<Rect> &_data, double _eps, int _mnpts) : data(_data) {
    C = -1;
    for (int i = 0; i < data.size(); i++) {
      labels[i] = -99;
    }
    eps = _eps;
    mnpts = _mnpts;
  }
  void run() {
    dp = new double[data.size() * data.size()];
    for (int i = 0; i < data.size(); i++) {
      for (int j = 0; j < data.size(); j++) {
        if (i == j)
          DP(i, j) = 0;
        else
          DP(i, j) = -1;
      }
    }
    for (int i = 0; i < data.size(); i++) {
      if (!isVisited(i)) {
        vector<int> neighbours = regionQuery(i);
        if (neighbours.size() < mnpts) {
          labels[i] = -1; // noise
        } else {
          C++;
          expandCluster(i, neighbours);
        }
      }
    }
    delete[] dp;
  }
  void expandCluster(int p, vector<int> neighbours) {
    labels[p] = C;
    for (int i = 0; i < neighbours.size(); i++) {
      if (!isVisited(neighbours[i])) {
        labels[neighbours[i]] = C;
        vector<int> neighbours_p = regionQuery(neighbours[i]);
        if (neighbours_p.size() >= mnpts) {
          expandCluster(neighbours[i], neighbours_p);
        }
      }
    }
  }

  bool isVisited(int i) { return labels[i] != -99; }

  vector<int> regionQuery(int p) {
    vector<int> res;
    for (int i = 0; i < data.size(); i++) {
      if (distanceFunc(p, i) <= eps) {
        res.push_back(i);
      }
    }
    return res;
  }

  double dist2d(Point2d a, Point2d b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
  }

  double distanceFunc(int ai, int bi) {
    if (DP(ai, bi) != -1)
      return DP(ai, bi);
    Rect a = data[ai];
    Rect b = data[bi];
    /*
    Point2d cena= Point2d(a.x+a.width/2,
                          a.y+a.height/2);
    Point2d cenb = Point2d(b.x+b.width/2,
                          b.y+b.height/2);
    double dist = sqrt(pow(cena.x-cenb.x,2) + pow(cena.y-cenb.y,2));
    DP(ai,bi)=dist;
    DP(bi,ai)=dist;*/
    Point2d tla = Point2d(a.x, a.y);
    Point2d tra = Point2d(a.x + a.width, a.y);
    Point2d bla = Point2d(a.x, a.y + a.height);
    Point2d bra = Point2d(a.x + a.width, a.y + a.height);

    Point2d tlb = Point2d(b.x, b.y);
    Point2d trb = Point2d(b.x + b.width, b.y);
    Point2d blb = Point2d(b.x, b.y + b.height);
    Point2d brb = Point2d(b.x + b.width, b.y + b.height);

    double minDist = 9999999;

    minDist = min(minDist, dist2d(tla, tlb));
    minDist = min(minDist, dist2d(tla, trb));
    minDist = min(minDist, dist2d(tla, blb));
    minDist = min(minDist, dist2d(tla, brb));

    minDist = min(minDist, dist2d(tra, tlb));
    minDist = min(minDist, dist2d(tra, trb));
    minDist = min(minDist, dist2d(tra, blb));
    minDist = min(minDist, dist2d(tra, brb));

    minDist = min(minDist, dist2d(bla, tlb));
    minDist = min(minDist, dist2d(bla, trb));
    minDist = min(minDist, dist2d(bla, blb));
    minDist = min(minDist, dist2d(bla, brb));

    minDist = min(minDist, dist2d(bra, tlb));
    minDist = min(minDist, dist2d(bra, trb));
    minDist = min(minDist, dist2d(bra, blb));
    minDist = min(minDist, dist2d(bra, brb));
    DP(ai, bi) = minDist;
    DP(bi, ai) = minDist;
    return DP(ai, bi);
  }

  vector<vector<Rect>> getGroups() {
    vector<vector<Rect>> ret;
    for (int i = 0; i <= C; i++) {
      ret.push_back(vector<Rect>());
      for (int j = 0; j < data.size(); j++) {
        if (labels[j] == i) {
          ret[ret.size() - 1].push_back(data[j]);
        }
      }
    }
    return ret;
  }
};

cv::Scalar HSVtoRGBcvScalar(int H, int S, int V) {
  int bH = H; // H component
  int bS = S; // S component
  int bV = V; // V component
  double fH, fS, fV;
  double fR, fG, fB;
  const double double_TO_BYTE = 255.0f;
  const double BYTE_TO_double = 1.0f / double_TO_BYTE;

  // Convert from 8-bit integers to doubles
  fH = (double)bH * BYTE_TO_double;
  fS = (double)bS * BYTE_TO_double;
  fV = (double)bV * BYTE_TO_double;

  // Convert from HSV to RGB, using double ranges 0.0 to 1.0
  int iI;
  double fI, fF, p, q, t;

  if (bS == 0) {
    // achromatic (grey)
    fR = fG = fB = fV;
  } else {
    // If Hue == 1.0, then wrap it around the circle to 0.0
    if (fH >= 1.0f)
      fH = 0.0f;

    fH *= 6.0;      // sector 0 to 5
    fI = floor(fH); // integer part of h (0,1,2,3,4,5 or 6)
    iI = (int)fH;   // " " " "
    fF = fH - fI;   // factorial part of h (0 to 1)

    p = fV * (1.0f - fS);
    q = fV * (1.0f - fS * fF);
    t = fV * (1.0f - fS * (1.0f - fF));

    switch (iI) {
    case 0:
      fR = fV;
      fG = t;
      fB = p;
      break;
    case 1:
      fR = q;
      fG = fV;
      fB = p;
      break;
    case 2:
      fR = p;
      fG = fV;
      fB = t;
      break;
    case 3:
      fR = p;
      fG = q;
      fB = fV;
      break;
    case 4:
      fR = t;
      fG = p;
      fB = fV;
      break;
    default: // case 5 (or 6):
      fR = fV;
      fG = p;
      fB = q;
      break;
    }
  }

  // Convert from doubles to 8-bit integers
  int bR = (int)(fR * double_TO_BYTE);
  int bG = (int)(fG * double_TO_BYTE);
  int bB = (int)(fB * double_TO_BYTE);

  // Clip the values to make sure it fits within the 8bits.
  if (bR > 255)
    bR = 255;
  if (bR < 0)
    bR = 0;
  if (bG > 255)
    bG = 255;
  if (bG < 0)
    bG = 0;
  if (bB > 255)
    bB = 255;
  if (bB < 0)
    bB = 0;

  // Set the RGB cvScalar with G B R, you can use this values as you want too..
  return cv::Scalar(bB, bG, bR); // R component
}

void callback(const sensor_msgs::Image::ConstPtr &msg) {
  cv_bridge::CvImagePtr cvPtr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat im;
  cv::threshold(cvPtr->image, im, 100, 255, CV_THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(im.clone(), contours, hierarchy, CV_RETR_LIST,
               CV_CHAIN_APPROX_SIMPLE);

  vector<Rect> boxes;
  for (size_t i = 0; i < contours.size(); i++) {
    Rect r = boundingRect(contours[i]);
    boxes.push_back(r);
  }
  DbScan dbscan(boxes, 20, 2);
  dbscan.run();
  // done, perform display

  Mat grouped = Mat::zeros(im.size(), CV_8UC3);
  vector<Scalar> colors;
  RNG rng(3);
  for (int i = 0; i <= dbscan.C; i++) {
    colors.push_back(HSVtoRGBcvScalar(rng(255), 255, 255));
  }
  for (int i = 0; i < dbscan.data.size(); i++) {
    Scalar color;
    if (dbscan.labels[i] == -1) {
      color = Scalar(128, 128, 128);
    } else {
      int label = dbscan.labels[i];
      color = colors[label];
    }
    putText(grouped, to_string(dbscan.labels[i]), dbscan.data[i].tl(),
            FONT_HERSHEY_COMPLEX, .5, color, 1);
    drawContours(grouped, contours, i, color, -1);
  }

  imshow("grouped", grouped);
  imwrite("c:/data/grouped.jpg", grouped);
  waitKey(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_obstacle_detectiont");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("hawk/object_detection", 1, callback);
  ros::spin();

  return 0;
}

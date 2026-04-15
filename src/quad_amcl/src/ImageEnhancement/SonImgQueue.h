#ifndef SONIMGQUEUE_H
#define SONIMGQUEUE_H

#include <opencv2/imgproc.hpp>
#include <vector>

#include "SonImg.h"

using namespace cv;
using namespace std;

class SonImgQueue
{
  vector<SonImg> imgs;

// ===== Navigation Stuff ======
  uint n=0, maxSize,index=0;

  uint prvId(uint id);
  uint nxtId(uint id);
  uint elementId(uint position);

public:
  SonImgQueue(uint maxSize=10u);
  uint size();
  bool empty();
  uint getMaxSize();

  SonImg & operator [](uint id);
  void addImg(SonImg &img);

  SonImg & last();

  void clear();
};

#endif // SONIMGQUEUE_H

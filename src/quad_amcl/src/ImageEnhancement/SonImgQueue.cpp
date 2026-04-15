#include "SonImgQueue.h"
#include <opencv2/highgui.hpp>
#include <iostream>

uint SonImgQueue::prvId(uint id)
{
  // Prevent overflow
  return uint((int(id)-1+maxSize)%maxSize);
}

uint SonImgQueue::nxtId(uint id)
{
  return uint((id+1)%maxSize);
}

uint SonImgQueue::elementId(uint position)
{
  if(n < maxSize)
    return position;
  else
    return uint((index+position)%maxSize);
}

uint SonImgQueue::size()
{
  return uint(n);
}

bool SonImgQueue::empty()
{
  return n==0;
}

uint SonImgQueue::getMaxSize()
{
  return maxSize;
}

SonImg &SonImgQueue::operator [](uint id)
{
  uint internal_id = elementId(id);
  return imgs[internal_id];
}

SonImgQueue::SonImgQueue(uint maxSize):
  imgs(maxSize),
  maxSize(maxSize)
{

}

/**
 * @brief SonImgQueue::addImg
 * @param img
 * @param H - Transform previous img to current img plane.
 */
void SonImgQueue::addImg(SonImg &img)
{
  imgs[uint(index)] = img; // Set new img

  index = nxtId(index);
  if(n < maxSize)
    n++;
}

SonImg &SonImgQueue::last()
{
  return imgs[prvId(index)];
}

void SonImgQueue::clear()
{
  index = 0;
  n = 0;
}

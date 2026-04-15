#include <opencv2/core.hpp>
#include "AerialImage.h"


bool AerialImage::loadYaml(const path &file)
{
    FileStorage fs;
    fs.open(file.string(), FileStorage::READ | FileStorage::FORMAT_YAML);

    if (!fs.isOpened())
    {
        cerr << "Failed to open " << file.string() << endl;
        return false;
    }

    const FileNode &nP1 = fs["p1"],
                   &nP2 = fs["p2"],
                   &nP3 = fs["p3"];

    // Leer coordenadas en metros (X, Y)
    UTMRef[0].x = double(nP1["x"]);
    UTMRef[0].y = double(nP1["y"]);
    
    imgRef[0].x = double(nP1["pix_x"]);
    imgRef[0].y = double(nP1["pix_y"]);
    
    UTMRef[1].x = double(nP2["x"]);
    UTMRef[1].y = double(nP2["y"]);
    
    imgRef[1].x = double(nP2["pix_x"]);
    imgRef[1].y = double(nP2["pix_y"]);

    UTMRef[2].x = double(nP3["x"]);
    UTMRef[2].y = double(nP3["y"]);
    
    imgRef[2].x = double(nP3["pix_x"]);
    imgRef[2].y = double(nP3["pix_y"]);


    
    // Calcular diferencias
    Point2d diffImg = imgRef[1] - imgRef[0];
    Point2d diffUTM = UTMRef[1] - UTMRef[0];
    
    // Calcular escala (metros a píxeles)
    UTM2Img_.x = diffImg.x / diffUTM.x;
    UTM2Img_.y = diffImg.y / diffUTM.y;

    // // Definir matriz 3x3
    // cv::Mat M = (cv::Mat_<double>(3,3) << 2.23875619e-14, -6.7, -4297.5,
    //                                       -6.7, -2.66197567e-15, 2513.5,
    //                                       0, 0, 1);

    // Definir punto
    // cv::Mat p = (cv::Mat_<double>(3,1) << 335.0, -705.0, 1.0);

    // Multiplicar
    // cv::Mat resultado = M * p;


    
    // DEBUG: Imprimir valores para verificar
    cout << "=== Calibración del Mapa ===" << endl;
    cout << "Punto 1: UTM(" << UTMRef[0].x << ", " << UTMRef[0].y 
         << ") -> Pixel(" << imgRef[0].x << ", " << imgRef[0].y << ")" << endl;
    cout << "Punto 2: UTM(" << UTMRef[1].x << ", " << UTMRef[1].y 
         << ") -> Pixel(" << imgRef[1].x << ", " << imgRef[1].y << ")" << endl;
    cout << "Punto 3: UTM(" << UTMRef[2].x << ", " << UTMRef[2].y 
         << ") -> Pixel(" << imgRef[2].x << ", " << imgRef[2].y << ")" << endl;


    // cout << "Matriz M =" << M << endl;
    // cout << "RESULTADO =" << resultado << endl;
    cout << "=========================" << endl;
    
    return true;
}

// bool AerialImage::loadYaml(const path &file)
// {
//   //  if(!is_regular_file(file))
//   //    return false;
//     //cout << "Load map config from " << file.string() << endl;

//     FileStorage fs;
//     fs.open(file.string(), FileStorage::READ |  FileStorage::FORMAT_YAML);

//     if (!fs.isOpened())
//     {
//         cerr << "Failed to open " << file.string() << endl;
//         return false;
//     }

//     const FileNode &nP1 = fs["p1"],
//                    &nP2 = fs["p2"];

//     // COMENTE ------------------------------------
//     // int zone1,zone2;
//     // Point2d p1((double(nP1["lat"])),double(nP1["long"])),
//     //         p2((double(nP2["lat"])),double(nP2["long"]));
//     // COMENTE ------------------------------------


//     // USAR DIRECTAMENTE como coordenadas en metros (x, y)
//     UTMRef[0].x = double(nP1["x"]);     // ← Cambiar "lat" por "x"
//     UTMRef[0].y = double(nP1["y"]);     // ← Cambiar "long" por "y"
    
//     imgRef[0].x = double(nP1["pix_x"]);
//     imgRef[0].y = double(nP1["pix_y"]);
    
//     UTMRef[1].x = double(nP2["x"]);
//     UTMRef[1].y = double(nP2["y"]);
    
//     imgRef[1].x = double(nP2["pix_x"]);
//     imgRef[1].y = double(nP2["pix_y"]);
    
    
//     // COMENTE ------------------------------------
//     // UTMRef[0] = latLon2UTM(p1,&zone1);

//     imgRef[0].x =  double(nP1["pix_x"]);
//     imgRef[0].y =  double(nP1["pix_y"]);

//     // COMENTE ------------------------------------
//     // UTMRef[1] = latLon2UTM(p2,&zone2);

//     imgRef[1].x =  double(nP2["pix_x"]);
//     imgRef[1].y =  double(nP2["pix_y"]);

//     //cout << UTMRef[0] << " zone " << zone1 << endl
//     //    << imgRef[0] << endl << endl
//     //     << UTMRef[1] << " zone " << zone2 << endl
//     //     << imgRef[1] << endl << endl;


//     // COMENTE ------------------------------------
//     // if(zone1 != zone2)
//     // {
//     //   cout << "Warnig: Coords in different zone!!!!!" << endl;
//     //   return false;
//     // }
//     // COMENTE ------------------------------------


//     Point2d diffImg = imgRef[1] - imgRef[0],
//             diffUTM = UTMRef[1] - UTMRef[0];

//     // Compute scale transform from UTM to pixel
//     // taking two reference points on both coordinate system

//     UTM2Img_.x = diffImg.x/diffUTM.x;
//     UTM2Img_.y = diffImg.y/diffUTM.y;

//     return true;
// }


AerialImage::AerialImage()
{

}

void AerialImage::resizeToMaxCols(int maxCols)
{
  double scale = double(maxCols)/mapImg.cols;

  resize(mapImg,mapImg,Size(), scale,scale);

  imgRef[0]*= scale;
  imgRef[1]*= scale;
  UTM2Img_*=scale;
}


/**
 * @brief AerialImage::getRectThatFitsIntoImg
 *  Giving a rect and an image, return an adjusted rect
 * that fits into the image. top,bottom, left and right
 * are output parameters that tells how many pixels
 * were missing on each direction. This information
 * can be used for a padding strategy.
 *  If the gave rect is completely out of the image
 * a empty rect is returned and -1 on the output
 * parameters.
 * @param r - Input - Initial rect.
 * @param top - Outout - changes on top of the rect.
 * @param bottom - Output - changes on bottom of the rect.
 * @param left - Outoput - changes on left of the rect.
 * @param right - Output - changens on right of the rect.
 * @return new rect that fits into the aerial image. An empty rect
 * is returned if it is not possible to fit the rect.
 */
Rect AerialImage::getTranslateRectToFit(const Rect &r, int &top, int &bottom, int &left, int &right) const
{
  top = bottom = left = right = 0;

  int minX = r.x, maxX = r.x + r.width,
      minY = r.y, maxY = r.y + r.height;

  if(maxX < 0 || minX > mapImg.cols ||
     maxY < 0 || minY > mapImg.rows)
  {
      top=bottom=left=right=-1;
      return Rect();
  }
  Rect fr(r);

  if(minX < 0)
  {
      fr.width+=minX; // Increased
      fr.x =0; // moved to right
      left=-minX;
  }

  if(maxX > mapImg.cols)
  {
      fr.width = mapImg.cols - fr.x;
      right = maxX - mapImg.cols;
  }

  if(minY < 0)
  {
      fr.height+=fr.y;
      fr.y = 0;
      top = -minY;
  }

  if(maxY > mapImg.rows)
  {
      fr.height = mapImg.rows - fr.y;
      bottom = maxY - mapImg.rows;
  }
  return fr;
}
bool AerialImage::loadMap(const path &mapFile)
{
  path yamlFile(mapFile),
       imgFile(mapFile);
  bool succes = false;

  yamlFile.replace_extension(".yaml");

  mapName = mapFile.stem().string();

  if(!loadYaml(yamlFile))
    return false;

  imgFile.replace_extension(".jpg");
  succes = is_regular(imgFile);

  // Try png
  if(!succes)
  {
    imgFile.replace_extension(".png");
    succes = is_regular(imgFile);
  }
  // Try jpeg
  if(!succes)
  {
    imgFile.replace_extension(".jpeg");
    succes = is_regular(imgFile);
  }
  if(!succes)
    return false;

  mapImg = imread(imgFile.string());
  if(mapImg.empty())
  {
    cerr << "Error: Could not open map img " << imgFile.string() <<endl;
    return false;
  }
  return true;
}

bool AerialImage::isOnMap(const Point2f &p)
{
  Point2i pImg = UTM2Img(p);
  if(pImg.x < 0 || pImg.x >= mapImg.cols ||
     pImg.y < 0 || pImg.y >= mapImg.rows)
    return false;
  return true;
}

bool AerialImage::isOnMapImg(const Point2f &pImg)
{
  if(pImg.x < 0 || pImg.x >= mapImg.cols ||
     pImg.y < 0 || pImg.y >= mapImg.rows)
    return false;
  return true;
}

Point2d AerialImage::UTM2Img(const Point2d &UTMp) const
{
  // Definir matriz 3x3
  cv::Mat M = (cv::Mat_<double>(3,3) << 2.23875619e-14, -6.7, -4297.5,
                                        -6.7, -2.66197567e-15, 2513.5,
                                        0, 0, 1);
  cv::Mat p = (cv::Mat_<double>(3, 1) << UTMp.x, UTMp.y, 1.0);


  cv::Mat resultado = M * p;

      
  // Extraer coordenadas de píxel
  double pixel_x = resultado.at<double>(0, 0);
  double pixel_y = resultado.at<double>(1, 0);
  return Point2d(pixel_x, pixel_y);
  // return Point2d (imgRef[0].x + (UTMp.x - UTMRef[0].x)*UTM2Img_.x,
  //                 imgRef[0].y + (UTMp.y - UTMRef[0].y)*UTM2Img_.y);

}

Point2d AerialImage::Img2UTM(const Point2d &ImgP) const
{
  // First convert img point to the origin (reference point 0)
  // Second apply the scale, now it is in UTM units and out of the center
  // Third translate to the UTM reference
  return Point2d (UTMRef[0].x + (ImgP.x - imgRef[0].x)/UTM2Img_.x,
      UTMRef[0].y + (ImgP.y - imgRef[0].y)/UTM2Img_.y);
}

Point2d AerialImage::UTM2ImgRatio() const
{

  double a = 2.23875619e-14;
  double b = -6.7;
  double c = -6.7;
  double d = -2.66197567e-15;

  double scale_x = sqrt(a*a + b*b);
  double scale_y = sqrt(c*c + d*d);

  return Point2d(scale_x, scale_y);
  // return UTM2Img_;
}

const Mat &AerialImage::getMapImg() const
{
  return mapImg;
}

string AerialImage::name()
{
  return mapName;
}

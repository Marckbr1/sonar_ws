#include "SonShape.h"

/**
 * @brief SonShape::getPoints - Return points on current heading
 * "heading" and position "p". All points are in UTM.
 * @param pts - Output transformed points
 * @param heading - Desired heading in rads
 * @param p - Desired position in meters (UTM).
 */
void SonShape::getPoints(vector<Point2d> &pts,double heading,const Point2d &p)
{
  // Apply rotation + translation
  pts.resize(m_pts.size());
  // ROS_INFO("m_pts size: %lu", m_pts.size());

  // ROS_INFO("Angulo: %.3f", heading);
  // ROS_INFO("Puntos = (%.3f, %.3f)", p.x, p.y);

  double s= sin(heading), c=cos(heading);

  for(uint i = 0 ; i < pts.size();i++)
  {
    // ROS_INFO("m_pts[%lu] = (%.3f, %.3f)", (unsigned long)i, m_pts[i].x, m_pts[i].y);
    Point2d &nP = pts[i];
    const Point2d & cP = m_pts[i];

    // We flip y signal because image y grows down
    // On UTM to img it will flip again
    nP.x =   cP.x*c - cP.y*s;
    nP.y = (cP.x*s + cP.y*c);

    nP.x +=  p.x;
    nP.y +=  p.y;

  }
}


SonShape::SonShape()
{

}

/**
 * @brief SonShape::initShape - Create the sonar field
 * of view polygon based on the sonar settings
 * @param opening - Sonar field of view opening in degrees.
 * @param maxRange - Sonar maximum range in meters.
 * @param minRange - Sonar minimum range in meters.
 */
void SonShape::initShape(double opening,
                         double maxRange, double minRange)
{
  int minRangeArcPoints=5, // Must be > 1
      maxRangeArcPoints=15, // Must be > 1
      pId = 0;

  m_range = maxRange;
  m_openning = opening;

  m_pts.resize(minRangeArcPoints+maxRangeArcPoints);

  double radBearing = opening*M_PI/180.0,
         radInc;

  // Resize vector of points
  m_pts.resize(minRangeArcPoints+maxRangeArcPoints);

  // Sonar origin is (0,0)

  // Maximum range arc (15 points)
  int kPts = 0;
  radInc = radBearing/(maxRangeArcPoints-1);

  for(double currentRad = -radBearing/2.0;
      kPts < maxRangeArcPoints;
      currentRad+= radInc) // Clockwise iteration
  {
      m_pts[pId++] = Point2d(sin(currentRad), cos(currentRad))*maxRange;
      kPts++;
  }

  // Minimum range arc (5 points)
  kPts = 0;
  radInc = radBearing/(minRangeArcPoints-1);
  for(double currentRad = radBearing/2.0;
      kPts < minRangeArcPoints;
      currentRad-= radInc) // Counter-Clockwise iteration
  {
      m_pts[pId++] = Point2d(sin(currentRad), cos(currentRad))*minRange;
      kPts++;
  }
  // Al final de initShape, imprime los primeros y últimos puntos
  ROS_INFO("SonShape inicializado: opening=%.1f°, maxRange=%.1f", opening, maxRange);
  ROS_INFO("Punto central (adelante): (%.2f, %.2f)", m_pts[7].x, m_pts[7].y);
  ROS_INFO("Punto de máxima distancia Y: %.2f", m_pts[7].y);


}

void SonShape::drawPoly(Mat &img,
                        const AerialImage &ai,
                        const Point2d &UTMPosition,
                        double heading,
                        const Scalar& color,
                        int thickness)
{
  // ROS_INFO("=== drawPoly DEBUG ===");
  // ROS_INFO("UTMPosition: (%.2f, %.2f)", UTMPosition.x, UTMPosition.y);
  // ROS_INFO("Heading: %.2f rad", heading);
  heading = heading - M_PI/2.0;
  vector<Point2d> newPts;
  getPoints(newPts,heading,UTMPosition);

  // cout << "=== PUNTOS DEL POLIGONO 01 (UTM) ===" << endl;
  // for(uint i = 0; i < newPts.size(); i++)
  // {
  //     cout << "Punto " << i << ": (" << newPts[i].x << ", " << newPts[i].y << ")" << endl;
  // }
  // cout << "================================" << endl;

  // ROS_INFO("Puntos UTM obtenidos: %lu", newPts.size());

  // Draw poly
  int numPts = newPts.size();
  Point pts[newPts.size()];

  // ROS_INFO("=== Conversión UTM->Píxel ===");
  // ROS_INFO("Vehículo UTM: (%.2f, %.2f)", UTMPosition.x, UTMPosition.y);

  // Point2d veh_pixel = ai.UTM2Img(UTMPosition);
  // ROS_INFO("Vehículo en píxeles: (%.2f, %.2f)", veh_pixel.x, veh_pixel.y);
  // // Convertir un punto que sabemos que está al NORTE del vehículo
  // // (por ejemplo, el punto con mayor Y)
  // Point2d punto_norte = UTMPosition;
  // punto_norte.y += 10;  // 10 metros al norte
  // Point2d norte_pixel = ai.UTM2Img(punto_norte);
  // ROS_INFO("Punto 10m al NORTE en píxeles: (%.2f, %.2f)", norte_pixel.x, norte_pixel.y);

  // // Convertir un punto al ESTE
  // Point2d punto_este = UTMPosition;
  // punto_este.x += 10;  // 10 metros al este
  // Point2d este_pixel = ai.UTM2Img(punto_este);
  // ROS_INFO("Punto 10m al ESTE en píxeles: (%.2f, %.2f)", este_pixel.x, este_pixel.y);


  for(uint i = 0; i < newPts.size(); i++)
  {
  //   ROS_INFO("newPts[%lu] = (%.3f, %.3f)", (unsigned long)i, newPts[i].x, newPts[i].y);
    const Point2d &imgP = ai.UTM2Img(newPts[i]); // convierto la coordenadas reales a pixeles
    // ROS_INFO("Dirección del sonar (relativa al vehículo): (%.2f, %.2f)", 
    //          direccion.x, direccion.y);
    // ROS_INFO("El sonar apunta hacia: %s", 
    //          fabs(direccion.y) > fabs(direccion.x) ? 
    //          (direccion.y > 0 ? "NORTE" : "SUR") :
    //          (direccion.x > 0 ? "ESTE" : "OESTE"));

    // ROS_INFO("imgPts[%lu] = (%.3f, %.3f)",
    //      (unsigned long)i,
    //      imgP.x,
    //      imgP.y);

    pts[i] = Point(round(imgP.x),
                   round(imgP.y));

  }

  int npts[] = {numPts};
  const Point* ppt[1] = { pts };

  // Draw poly on aerial image
  polylines(img,
            ppt,npts,
            1,true,
            color,thickness);
}



Mat SonShape::cropSonShape(const AerialImage &ai,
                          const Point2d &UTMPosition,
                          double heading)
{
 vector<Point2d> newPts;
 heading = heading - M_PI/2.0;
 getPoints(newPts, heading, UTMPosition);



  // cout << "=== PUNTOS DEL POLIGONO (UTM) ===" << endl;
  // for(uint i = 0; i < newPts.size(); i++)
  // {
  //     cout << "Punto " << i << ": (" << newPts[i].x << ", " << newPts[i].y << ")" << endl;
  // }
  // cout << "================================" << endl;

 int numPts = newPts.size();
 vector<Point> pts(numPts);
 Point minP, maxP;


 // Convertir puntos UTM a píxeles
 for(uint i = 0; i < newPts.size(); i++)
 {
   const Point2d &imgP = ai.UTM2Img(newPts[i]);
   Point &nP = pts[i];


   nP = Point(round(imgP.x), round(imgP.y));
   // IMPRIMIR CADA PUNTO CONVERTIDO
  //  cout << "Punto " << i << ": UTM(" << newPts[i].x << ", " << newPts[i].y 
  //       << ") -> Pixel(" << nP.x << ", " << nP.y << ")" << endl;

   if(i == 0)
   {
     minP = maxP = nP;
   }
   else
   {
     if(minP.x > nP.x) minP.x = nP.x;
     if(minP.y > nP.y) minP.y = nP.y;
     if(maxP.x < nP.x) maxP.x = nP.x;
     if(maxP.y < nP.y) maxP.y = nP.y;
   }
 }



//  // ============================================================
//  // IMPRIMIR RECTÁNGULO DELIMITADOR (sin margen)
//  // ============================================================
//  cout << "\n=== RECTANGULO DELIMITADOR (sin margen) ===" << endl;
//  cout << "minP: (" << minP.x << ", " << minP.y << ")" << endl;
//  cout << "maxP: (" << maxP.x << ", " << maxP.y << ")" << endl;
//  cout << "width: " << (maxP.x - minP.x) << ", height: " << (maxP.y - minP.y) << endl;
//  cout << "===========================================" << endl;

 // Calcular el centro del sonar (posición del robot)
 Point2d sonarPositionOnImg = ai.UTM2Img(UTMPosition);
  // Calcular rectángulo delimitador con margen
 int margin = 100;
 Rect rect(minP, maxP);
 rect.x -= margin;
 rect.y -= margin;
 rect.width += 2 * margin;
 rect.height += 2 * margin;

 cout << "\n=== RECTANGULO CON MARGEN (" << margin << " pixeles) ===" << endl;
 cout << "x: " << rect.x << ", y: " << rect.y << endl;
 cout << "width: " << rect.width << ", height: " << rect.height << endl;
 cout << "esquina superior izquierda: (" << rect.x << ", " << rect.y << ")" << endl;
 cout << "esquina inferior derecha: (" << rect.x + rect.width << ", " << rect.y + rect.height << ")" << endl;

 // Asegurar que el rectángulo esté dentro de la imagen
 Rect imageRect(0, 0, ai.getMapImg().cols, ai.getMapImg().rows);
 Rect rectThatFits = rect & imageRect;


 if(rectThatFits.width <= 0 || rectThatFits.height <= 0)
 {
     cout << "Sat image crop error!" << endl;
     return Mat();
 }


 // Calcular padding
 int top = max(0, rect.y - rectThatFits.y);
 int bottom = max(0, (rect.y + rect.height) - (rectThatFits.y + rectThatFits.height));
 int left = max(0, rect.x - rectThatFits.x);
 int right = max(0, (rect.x + rect.width) - (rectThatFits.x + rectThatFits.width));


 // Recortar la imagen
 Mat cropThatFits = ai.getMapImg()(rectThatFits);


 // Aplicar padding
 Mat sonarFoVRect;
 if(top > 0 || bottom > 0 || left > 0 || right > 0)
 {
   copyMakeBorder(cropThatFits, sonarFoVRect,
                  top, bottom, left, right,
                  BORDER_REFLECT_101);
 }
 else
 {
   sonarFoVRect = cropThatFits;
 }


 // Ajustar puntos al nuevo sistema de coordenadas
 vector<Point> adjustedPts(pts.size());
 for(size_t i = 0; i < pts.size(); i++)
 {
   adjustedPts[i] = Point(pts[i].x - rect.x + left,
                          pts[i].y - rect.y + top);
 }


 // Posición del sonar en el sistema de coordenadas recortado
 Point2d sonarPosOnCrop(sonarPositionOnImg.x - rect.x + left,
                        sonarPositionOnImg.y - rect.y + top);


 // === ENDEREZAR LA IMAGEN (rotación inversa) ===
 // Rotar la imagen para que el sonar siempre apunte hacia arriba
 double headingDeg = heading * 105.0 / M_PI;
  // Matriz de rotación (rotación negativa para enderezar)
 Mat rotMatrix = getRotationMatrix2D(sonarPosOnCrop, -headingDeg, 1.0);
  // Calcular nuevo tamaño después de rotar
 Rect boundingRot = RotatedRect(sonarPosOnCrop, sonarFoVRect.size(), -headingDeg).boundingRect();
 Mat rotated;
 warpAffine(sonarFoVRect, rotated, rotMatrix, boundingRot.size(), INTER_LINEAR);
  // Ajustar los puntos rotados
 vector<Point> rotatedPts(adjustedPts.size());
 for(size_t i = 0; i < adjustedPts.size(); i++)
 {
   // Transformar cada punto con la matriz de rotación
   rotatedPts[i].x = rotMatrix.at<double>(0,0) * adjustedPts[i].x +
                     rotMatrix.at<double>(0,1) * adjustedPts[i].y +
                     rotMatrix.at<double>(0,2);
   rotatedPts[i].y = rotMatrix.at<double>(1,0) * adjustedPts[i].x +
                     rotMatrix.at<double>(1,1) * adjustedPts[i].y +
                     rotMatrix.at<double>(1,2);
 }
  // Posición del sonar en la imagen rotada
 Point2d sonarPosOnRotated;
 sonarPosOnRotated.x = rotMatrix.at<double>(0,0) * sonarPosOnCrop.x +
                       rotMatrix.at<double>(0,1) * sonarPosOnCrop.y +
                       rotMatrix.at<double>(0,2);
 sonarPosOnRotated.y = rotMatrix.at<double>(1,0) * sonarPosOnCrop.x +
                       rotMatrix.at<double>(1,1) * sonarPosOnCrop.y +
                       rotMatrix.at<double>(1,2);


 // Crear máscara con la forma del sonar (ahora enderezada)
 Mat mask(rotated.size(), CV_8UC1, Scalar(0));
 vector<vector<Point>> contours = {rotatedPts};
 fillPoly(mask, contours, Scalar(255));


 // Aplicar máscara
 Mat result;
 rotated.copyTo(result, mask);


 // Recortar para eliminar bordes negros
 Mat gray;
 cvtColor(result, gray, COLOR_BGR2GRAY);
 vector<Point> nonZeroPoints;
 findNonZero(gray, nonZeroPoints);
  if(!nonZeroPoints.empty())
 {
   Rect contentRect = boundingRect(nonZeroPoints);
   // Añadir pequeño margen
   contentRect.x = max(0, contentRect.x - 10);
   contentRect.y = max(0, contentRect.y - 10);
   contentRect.width = min(result.cols - contentRect.x, contentRect.width + 20);
   contentRect.height = min(result.rows - contentRect.y, contentRect.height + 20);
   result = result(contentRect).clone();
 }


 return result;
}



// Mat SonShape::cropSonShape(const AerialImage &ai,
//                            const Point2d &UTMPosition,
//                            double heading)
// {
//   vector<Point2d> newPts;
//   getPoints(newPts,heading,UTMPosition);

//   int numPts = newPts.size();
//   Point pts[newPts.size()],
//       minP,maxP;


//   for(uint i = 0; i < newPts.size(); i++)
//   {
//     const Point2d &imgP = ai.UTM2Img(newPts[i]);
//     Point &nP = pts[i];

//     nP = Point(round(imgP.x),
//                round(imgP.y));
//     if(i==0)
//     {
//       minP = maxP = nP;
//     }else
//     {
//       if(minP.x > nP.x) minP.x = nP.x;
//       if(minP.y > nP.y) minP.y = nP.y;
//       if(maxP.x < nP.x) maxP.x = nP.x;
//       if(maxP.y < nP.y) maxP.y = nP.y;
//     }
//   }



//   // Point2d imgPos = ai.UTM2Img(UTMPosition);

//   // cout << "Image size: " << ai.getMapImg().cols << " x " << ai.getMapImg().rows << endl;
//   // cout << "Sonar pixel position: " << imgPos.x << " , " << imgPos.y << endl;


//   // // AQUÍ VA EL CÓDIGO NUEVO
//   // if(imgPos.x < 0 || imgPos.x >= ai.getMapImg().cols ||
//   //   imgPos.y < 0 || imgPos.y >= ai.getMapImg().rows)
//   // {
//   //     cout << "Sonar outside map, skipping frame" << endl;
//   //     return Mat();
//   // }



//   // Crop sat img
//   // A boding box rect with 3 extra pixels each side
//   Rect rect(minP,maxP);
//   rect.x-=3; rect.y-=3; rect.width+=6; rect.height+=6;

//   // Find roi that fits
//   int top,bottom, left,right;
//   Rect rectThatFits = ai.getTranslateRectToFit(rect,
//                                              top,bottom,
//                                              left,right);

//   if(rectThatFits.width==0 || rectThatFits.height ==0)
//   {
//       cout << "Sat image crop error! Desired crop out of the map!!" << endl;
//       return Mat();
//   }

//   Mat sonarFoVRect; // Rect crop from the map

//   // Fill the missing part with padding
//   Mat cropThatFits = ai.getMapImg()(rectThatFits);

//   copyMakeBorder(cropThatFits,sonarFoVRect,
//                  top,bottom,left,right,
//                  BORDER_REFLECT_101);

//   // Transform the sonar FoV poly in an image mask
//   Mat sonarMask(rect.height, rect.width,CV_8UC1,Scalar(0));

//   int npts[] = {numPts};
//   const Point* ppt[1] = { pts };
//   fillPoly(sonarMask,ppt,npts,1,Scalar(255),
//            LINE_8,0,Point(-rect.x,-rect.y));

//   // Create the image correspondent of the sonar FoV
//   Mat sonFoVImg;
//   sonarFoVRect.copyTo(sonFoVImg,sonarMask);

//   // Compute Sonar Position on each coordinate system
//   double FoVRad = m_openning*M_PI/180.0;

//   Point2d sonarPositionOnImg(ai.UTM2Img(UTMPosition)),
//           sonarPositionOnResult( sonarPositionOnImg.x - rect.x,
//                                  sonarPositionOnImg.y - rect.y),
//           sonarFoVSize(2*m_range*cos(M_PI_2-(FoVRad/2.0)), // width
//                        m_range // Height
//                        );

//   // ===== Rotating the FoV image regarding sonar heading =============
//   Point2d Utm2Img = ai.UTM2ImgRatio();
//   Size finalImgSize(abs(int(sonarFoVSize.x*Utm2Img.x)),
//                     abs(int(sonarFoVSize.y*Utm2Img.y)));

//   Mat afimTransformMatrix = getRotationMatrix2D(sonarPositionOnResult,
//                                                 heading*180.0/M_PI,1.0);

//   // Translation Correction
//   afimTransformMatrix.at<double>(0,2) += -sonarPositionOnResult.x+finalImgSize.width/2;
//   afimTransformMatrix.at<double>(1,2) += -sonarPositionOnResult.y+finalImgSize.height;

//   // Apply affine transform and warp the image
//   warpAffine( sonFoVImg,
//               sonFoVImg,
//               afimTransformMatrix,
//               finalImgSize,
//               INTER_NEAREST
//              );

//   return sonFoVImg;
// }

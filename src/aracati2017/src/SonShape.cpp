#include "SonShape.h"

#include <opencv2/opencv.hpp>
using namespace cv;


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
  // ROS_INFO("SonShape inicializado: opening=%.1f°, maxRange=%.1f", opening, maxRange);
  // ROS_INFO("Punto central (adelante): (%.2f, %.2f)", m_pts[7].x, m_pts[7].y);
  // ROS_INFO("Punto de máxima distancia Y: %.2f", m_pts[7].y);


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
  // ROS_INFO("=== drawPoly DEBUG ===");
  // ROS_INFO("UTMPosition: (%.2f, %.2f)", UTMPosition.x, UTMPosition.y);
  // ROS_INFO("Heading: %.2f rad", heading);
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
  // Náutico → matemático
  const double headingMath = heading - M_PI/2.0;

  // Posición del vehículo en píxeles y ratio UTM→imagen
  const Point2d centerImg  = ai.UTM2Img(UTMPosition);
  const Point2d utm2Img    = ai.UTM2ImgRatio();

  // Recorte cuadrado fijo centrado en el vehículo
  const int halfW = abs(int(m_range * utm2Img.x));
  const int halfH = abs(int(m_range * utm2Img.y));

  Rect rect(int(centerImg.x) - halfW - 3,
            int(centerImg.y) - halfH - 3,
            halfW * 2 + 6,
            halfH * 2 + 6);

  // Ajustar si se sale del mapa
  int top, bottom, left, right;
  const Rect rectThatFits = ai.getTranslateRectToFit(rect, top, bottom, left, right);

  if(rectThatFits.width == 0 || rectThatFits.height == 0) {
    cout << "Sat image crop error! Desired crop out of the map!!" << endl;
    return Mat();
  }

  // Recortar y rellenar bordes
  Mat sonarFoVRect;
  copyMakeBorder(ai.getMapImg()(rectThatFits), sonarFoVRect,
                 top, bottom, left, right,
                 BORDER_REFLECT_101);

  // Máscara del FoV rotado según heading
  vector<Point2d> newPts;
  getPoints(newPts, headingMath, UTMPosition);

  const int numPts = newPts.size();
  vector<Point> pts(numPts);
  for(int i = 0; i < numPts; i++) {
    const Point2d& imgP = ai.UTM2Img(newPts[i]);
    pts[i] = Point(round(imgP.x), round(imgP.y));
  }

  Mat sonarMask(rect.height, rect.width, CV_8UC1, Scalar(0));
  const Point* ppt[1] = { pts.data() };
  int npts[] = { numPts };
  fillPoly(sonarMask, ppt, npts, 1, Scalar(255),
           LINE_8, 0, Point(-rect.x, -rect.y));

  // Aplicar máscara
  Mat sonFoVImg;
  sonarFoVRect.copyTo(sonFoVImg, sonarMask);

  // Corregir orientación → siempre norte arriba
  const double angleDeg = -(heading) * 180.0 / M_PI; // heading original sin -π/2
  const Point2f center(sonFoVImg.cols / 2.0f, sonFoVImg.rows / 2.0f);
  Mat sonFoVImgNorth;
  warpAffine(sonFoVImg, sonFoVImgNorth,
             getRotationMatrix2D(center, angleDeg, 1.0),
             sonFoVImg.size(), INTER_LINEAR);

  // Devolver solo la mitad superior (área del FoV)
  return sonFoVImgNorth(Rect(0, 0, sonFoVImgNorth.cols, sonFoVImgNorth.rows / 2));
}









// Mat SonShape::cropSonShape(const AerialImage &ai,
//                            const Point2d &UTMPosition,
//                            double heading)
// {
//   heading = heading - M_PI/2.0;
//   // Convertir posición UTM a píxel
//   Point2d centerImg = ai.UTM2Img(UTMPosition);
//   Point2d Utm2Img = ai.UTM2ImgRatio();

//   // Tamaño fijo basado en el rango del sonar
//   int halfW = abs(int(m_range * Utm2Img.x));
//   int halfH = abs(int(m_range * Utm2Img.y));

//   // Recorte fijo centrado en el vehículo — nunca rota
//   Rect rect(
//     int(centerImg.x) - halfW,
//     int(centerImg.y) - halfH,
//     halfW * 2,
//     halfH * 2
//   );
//   rect.x -= 3; rect.y -= 3; rect.width += 6; rect.height += 6; 

//   int top, bottom, left, right;
//   Rect rectThatFits = ai.getTranslateRectToFit(rect, top, bottom, left, right); // Ajuste del rectangulo 

//   if(rectThatFits.width == 0 || rectThatFits.height == 0) {
//     cout << "Sat image crop error! Desired crop out of the map!!" << endl;
//     return Mat();
//   }

//   Mat cropThatFits = ai.getMapImg()(rectThatFits); // imagen rectangular 

//   // imshow("crop", cropThatFits);
//   // waitKey(0);  // espera una tecla

//   Mat sonarFoVRect;

//   // imshow("crop", sonarFoVRect); // Es vacio
//   // waitKey(0);  // espera una tecla
  
//   copyMakeBorder(cropThatFits, sonarFoVRect,
//                  top, bottom, left, right,
//                  BORDER_REFLECT_101);
  
//   // imshow("rect", sonarFoVRect);
//   // waitKey(0);


//   // Máscara con el FoV real (rotado según heading)
//   vector<Point2d> newPts;
//   getPoints(newPts, heading, UTMPosition);

//   int numPts = newPts.size();
//   Point pts[numPts];
//   for(uint i = 0; i < newPts.size(); i++)
//   {
//     const Point2d &imgP = ai.UTM2Img(newPts[i]);
//     pts[i] = Point(round(imgP.x), round(imgP.y));
//   }

//   Mat sonarMask(rect.height, rect.width, CV_8UC1, Scalar(0)); // Es una imagen de color negro
//   int npts[] = {numPts};
//   const Point* ppt[1] = { pts };
//   fillPoly(sonarMask, ppt, npts, 1, Scalar(255),
//            LINE_8, 0, Point(-rect.x, -rect.y));
  
//   // 255 blanco
//   // 0 negro
//   // 

//   // Después de fillPoly
//   // imshow("Sonar Mask", sonarMask);
//   // waitKey(0);
//   // ROS_INFO("Heading: %.2f rad", heading);
//   // ROS_INFO("Otro Heading: %.2f rad", heading - M_PI/2.0);
//   // Aplicar máscara — recorte siempre fijo, solo la máscara rota
//   Mat sonFoVImg;
//   sonarFoVRect.copyTo(sonFoVImg, sonarMask);

//   // Mat sonFoVImg;
//   // sonarFoVRect.copyTo(sonFoVImg, sonarMask);

//   // heading ya tiene aplicado -M_PI/2 al inicio de la función
//   // entonces la corrección es exactamente -heading convertido a grados
//   double angle_deg = -(heading + M_PI/2.0) * 180.0 / M_PI;

//   Point2d center(sonFoVImg.cols / 2.0, sonFoVImg.rows / 2.0);
//   Mat rotMat = cv::getRotationMatrix2D(center, angle_deg, 1.0);

//   Mat sonFoVImgNorth;
//   warpAffine(sonFoVImg, sonFoVImgNorth, rotMat, 
//             sonFoVImg.size(), INTER_LINEAR);

//   // return sonFoVImgNorth;

//   Rect roi(0, 0, sonFoVImgNorth.cols, sonFoVImgNorth.rows / 2);
//   Mat half = sonFoVImgNorth(roi);

//   return half;




// }





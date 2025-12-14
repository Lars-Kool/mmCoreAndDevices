////////////////////////////////////////////////////////////////////////////////
// FILE:          DemoGalvo.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   A demo implementation of the CGalvoBase class. It
//                simulates a pressure pump device, and enables testing of
//                the rest of the system without the need to connect to the
//                actual hardware.
//                
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 06/08/2005
//                
//                Karl Hoover (stuff such as programmable CCD size  & the various image processors)
//                Arther Edelstein ( equipment error simulation)
//
// COPYRIGHT:     University of California, San Francisco, 2006-2015
//                100X Imaging Inc, 2008
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "DeviceBase.h"
#include "DemoHub.h"
#include "DemoCamera.h"

class ImgManipulator 
{
   public:
      virtual int ChangePixels(ImgBuffer& img) = 0;
};

struct Point
{
   public:
      Point(int lx, int ly) {x = lx; y = ly;};
   int x;
   int y;
};

struct PointD
{
   public:
      PointD(double lx, double ly) {x = lx; y = ly;};
   double x;
   double y;
};

class DemoGalvo : public CGalvoBase<DemoGalvo>, ImgManipulator
{
public:
   DemoGalvo();
   ~DemoGalvo();
      
   // MM::Device API
   bool Busy() {return busy_;}
   void GetName(char* pszName) const;
   int Initialize();
   int Shutdown(){initialized_ = false; return DEVICE_OK;}

   // MM::Galvo API
   int PointAndFire(double x, double y, double pulseTime_us); 
   int SetSpotInterval(double pulseTime_us);
   int SetPosition(double x, double y);
   int GetPosition(double& x, double& y);
   int SetIlluminationState(bool on);
   int AddPolygonVertex(int polygonIndex, double x, double y);
   int DeletePolygons();
   int LoadPolygons();
   int SetPolygonRepetitions(int repetitions);
   int RunPolygons();
   int RunSequence();
   int StopSequence();
   int GetChannel(char* channelName);                         

   double GetXRange();                         
   double GetYRange(); 

   int ChangePixels(ImgBuffer& img);
   static bool PointInTriangle(Point p, Point p0, Point p1, Point p2);

private:

   CDemoCamera* demoCamera_ = nullptr;
   unsigned short gaussianMask_[10][10]{};

   double GaussValue(double amplitude, double sigmaX, double sigmaY, int muX, int muY, int x, int y);
   Point GalvoToCameraPoint(PointD GalvoPoint, ImgBuffer& img);
   void GetBoundingBox(std::vector<Point>& vertex, std::vector<Point>& bBox);
   bool InBoundingBox(std::vector<Point> boundingBox, Point testPoint);

   std::map<int, std::vector<PointD> > vertices_;
   MM::MMTime pfExpirationTime_;
   bool initialized_ = false;
   bool busy_ = false;
   bool illuminationState_ = false;
   bool pointAndFire_ = false;
   bool runROIS_ = false;
   double xRange_ = 10.0;
   double yRange_ = 10.0;
   double currentX_ = 0.0;
   double currentY_ = 0.0;
   int offsetX_ = 20;
   double vMaxX_ = 10.0;
   int offsetY_ = 15;
   double vMaxY_ = 10.0;
   double pulseTime_Us_ = 100000.0;
};


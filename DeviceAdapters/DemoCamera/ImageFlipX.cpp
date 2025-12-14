////////////////////////////////////////////////////////////////////////////////
// FILE:          ImageFlipX.cpp
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   An implementation of an CImageProcessor that flips images
//                horizontally.
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

#include "DemoHub.h"
#include "ImageFlipX.h"

extern const char* g_ImageFlipXName;

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void ImageFlipX::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_ImageFlipXName);
}

// Initialize the processor and its properties.
int ImageFlipX::Initialize()
{
   CPropertyAction* pAct = new CPropertyAction (this, &ImageFlipX::OnPerformanceTiming);
   int ret = CreateFloatProperty("PeformanceTiming (microseconds)", 0, true, pAct);
   if (DEVICE_OK != ret) { return ret; }

   initialized_ = true;
   return DEVICE_OK;
}

// Shutdown the processor. Since no resources are used, there is nothing to do.
int ImageFlipX::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::ImageProcessor API
////////////////////////////////////////////////////////////////////////////////

// Process the image by flipping it horizontally.
// The bytebuffer is cast to different pixel types depending on the byteDepth parameter.
int ImageFlipX::Process(unsigned char *pBuffer, unsigned int width, unsigned int height, unsigned int byteDepth)
{
   if(busy_)
      return DEVICE_ERR;

   int ret = DEVICE_OK;
 
   busy_ = true;
   performanceTiming_ = MM::MMTime(0.);
   MM::MMTime  s0 = GetCurrentMMTime();


   if( sizeof(unsigned char) == byteDepth)
   {
      ret = Flip( (unsigned char*)pBuffer, width, height);
   }
   else if( sizeof(unsigned short) == byteDepth)
   {
      ret = Flip( (unsigned short*)pBuffer, width, height);
   }
   else if( sizeof(unsigned long) == byteDepth)
   {
      ret = Flip( (unsigned long*)pBuffer, width, height);
   }
   else if( sizeof(unsigned long long) == byteDepth)
   {
      ret =  Flip( (unsigned long long*)pBuffer, width, height);
   }
   else
   {
      ret =  DEVICE_NOT_SUPPORTED;
   }

   performanceTiming_ = GetCurrentMMTime() - s0;
   busy_ = false;

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action handlers
////////////////////////////////////////////////////////////////////////////////

// Read-only property that returns the time taken to perform the last image flip.
int ImageFlipX::OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct)
{

   if (eAct == MM::BeforeGet)
   {
      pProp->Set( performanceTiming_.getUsec());
   }
   else if (eAct == MM::AfterSet)
   {
      // -- it's ready only!
   }

   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////

// Template function that flips the image horizontally for different pixel types.
template <typename PixelType>
int ImageFlipX::Flip(PixelType* pI, unsigned int width, unsigned int height)
{
   PixelType tmp;
   int ret = DEVICE_OK;
   for (unsigned long iy = 0; iy < height; ++iy)
   {
		// bitshift used as division by 2 rounded down
      for (unsigned long ix = 0; ix < (width >> 1) ; ++ix)
      {
         tmp = pI[ix + iy * width];
         pI[ix + iy * width] = pI[width - 1 - ix + iy * width];
         pI[width - 1 - ix + iy * width] = tmp;
      }
   }
   return ret;
}

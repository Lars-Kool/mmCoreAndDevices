////////////////////////////////////////////////////////////////////////////////
// FILE:          ImageMedianFilter.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   An implementation of an CImageProcessor applies a Median filter.
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
#include "ImageMedianFilter.h"

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

// Copies the device name to the provided buffer.
void ImageMedianFilter::GetName(char* Name) const
{
   CDeviceUtils::CopyLimitedString(Name, g_ImageMedianFilterName);
}

// Initialize the processor and its timing property.
int ImageMedianFilter::Initialize()
{
   CPropertyAction* pAct = new CPropertyAction (this, &ImageMedianFilter::OnPerformanceTiming);
   int ret = CreateFloatProperty("PeformanceTiming (microseconds)", 0, true, pAct);
   if (DEVICE_OK != ret) { return ret; }
    
   // LK: This log message was a read-only property before. Not sure that is a behavior
   // we want to promote
   LogMessage("BEWARE, THIS FILTER MODIFIES DATA, EACH PIXEL IS REPLACED BY 3X3 NEIGHBORHOOD MEDIAN");
   initialized_ = true;
	return DEVICE_OK;
}

int ImageMedianFilter::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::ImageProcessor API
////////////////////////////////////////////////////////////////////////////////

// Process the image buffer by applying a median filter.
// The buffer is cast to different pixel types depending on the byteDepth parameter.
int ImageMedianFilter::Process(unsigned char *pBuffer, unsigned int width, unsigned int height, unsigned int byteDepth)
{
   if (busy_) {
      return DEVICE_ERR;
   }

   int ret = DEVICE_OK;
 
   busy_ = true;
   performanceTiming_ = MM::MMTime(0.);
   MM::MMTime  s0 = GetCurrentMMTime();

   if (sizeof(unsigned char) == byteDepth)
   {
      ret = Filter((unsigned char*)pBuffer, width, height);
   }
   else if (sizeof(unsigned short) == byteDepth)
   {
      ret = Filter((unsigned short*)pBuffer, width, height);
   }
   else if (sizeof(unsigned long) == byteDepth)
   {
      ret = Filter((unsigned long*)pBuffer, width, height);
   }
   else if (sizeof(unsigned long long) == byteDepth)
   {
      ret = Filter((unsigned long long*)pBuffer, width, height);
   }
   else
   {
      ret = DEVICE_NOT_SUPPORTED;
   }

   performanceTiming_ = GetCurrentMMTime() - s0;
   busy_ = false;

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action handlers
////////////////////////////////////////////////////////////////////////////////

// Read-only property returning the time taken to perform the last image median filter.
int ImageMedianFilter::OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct)
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

// Template function that applies a 3x3 median filter to the image.
template <typename PixelType>
int ImageMedianFilter::Filter(PixelType* pI, unsigned int width, unsigned int height)
{
      int ret = DEVICE_OK;
      int x[9];
      int y[9];

      const unsigned long thisSize = sizeof(*pI)*width*height;
      if( thisSize != sizeOfSmoothedIm_)
      {
         if(NULL!=pSmoothedIm_)
         {
            sizeOfSmoothedIm_ = 0;
            free(pSmoothedIm_);
         }
         // malloc is faster than new...
         pSmoothedIm_ = (PixelType*)malloc(thisSize);
         if(NULL!=pSmoothedIm_)
         {
            sizeOfSmoothedIm_ = thisSize;
         }
      }

      PixelType* pSmooth = (PixelType*) pSmoothedIm_;

      if(NULL != pSmooth)
      {
      /*Apply 3x3 median filter to reduce shot noise*/
      for (unsigned int i=0; i<width; i++) {
         for (unsigned int j=0; j<height; j++) {
            x[0]=i-1;
            y[0]=(j-1);
            x[1]=i;
            y[1]=(j-1);
            x[2]=i+1;
            y[2]=(j-1);
            x[3]=i-1;
            y[3]=(j);
            x[4]=i;
            y[4]=(j);
            x[5]=i+1;
            y[5]=(j);
            x[6]=i-1;
            y[6]=(j+1);
            x[7]=i;
            y[7]=(j+1);
            x[8]=i+1;
            y[8]=(j+1);
            // truncate the median filter window  -- duplicate edge points
            // this could be more efficient, we could fill in the interior image [1,w0-1]x[1,h0-1] then explicitly fill in the edge pixels.
            // also the temporary image could be as small as 2 rasters of the image
            for(int ij =0; ij < 9; ++ij)
            {
               if( x[ij] < 0)
                  x[ij] = 0;
               else if( int(width-1) < x[ij])
                  x[ij] = int(width-1);
               if( y[ij] < 0)
                  y[ij] = 0;
               else if( int(height-1) < y[ij])
                  y[ij] = (int)(height-1);
            }
            std::vector<PixelType> windo;
            for(int ij = 0; ij < 9; ++ij)
            {
               windo.push_back(pI[ x[ij] + width*y[ij]]);
            }
            pSmooth[i + j*width] = FindMedian(windo);
         }
      }

      memcpy( pI, pSmoothedIm_, thisSize);
      }
      else
         ret = DEVICE_ERR;

      return ret;
   }

// Find the median value in the vector of values.
// NOTE: this utility MODIFIES the argument, make a copy yourself if you want
// the original data preserved.
template <class U>
U FindMedian(std::vector<U>& values)
{
   std::sort(values.begin(), values.end());
   return values[(values.size())>>1];
}

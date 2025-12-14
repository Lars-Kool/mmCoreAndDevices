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

#pragma once

//////////////////////////////////////////////////////////////////////////////
// K.H.
//////////////////////////////////////////////////////////////////////////////
class ImageMedianFilter : public CImageProcessorBase<ImageMedianFilter>
{
public:
   ImageMedianFilter ()
   {
      // parent ID display
      CreateHubIDProperty();
   };
   ~ImageMedianFilter () { if(0!=pSmoothedIm_) free(pSmoothedIm_); };

	// MM::Device API
   void GetName(char* Name) const;
   bool Busy(void) { return busy_;};
   int Initialize();
   int Shutdown();

	// MM::ImageProcessor API
   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

	// MM Action handlers
   int OnPerformanceTiming(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
	// Utility functions
   template <typename PixelType>
   int Filter(PixelType* pI, unsigned int width, unsigned int height);

   template <class U>
   U FindMedian(std::vector<U>& values);

	// Data members
   bool busy_ = false;
	bool initialized_ = false;
   MM::MMTime performanceTiming_;
   void*  pSmoothedIm_ = nullptr;
   unsigned long sizeOfSmoothedIm_ = 0;
};


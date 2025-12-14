////////////////////////////////////////////////////////////////////////////////
// FILE:          ImageTranspose.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   An implementation of an CImageProcessor that transposes images.
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

////////////////////////////////////////////////////////////////////////////////
// K.H.
////////////////////////////////////////////////////////////////////////////////
class ImageTranspose : public CImageProcessorBase<ImageTranspose>
{
public:
   ImageTranspose ()
   {
      // parent ID display
      CreateHubIDProperty();
   }
   ~ImageTranspose () {if( NULL!= pTemp_) free(pTemp_); tempSize_=0;  }

	// MM::Device API
   void GetName(char* name) const;
   bool Busy(void) { return busy_;};
   int Initialize();
   int Shutdown();

	// MM::ImageProcessor API
   int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth);

	// MM Action handlers
   int OnInPlaceAlgorithm(MM::PropertyBase* pProp, MM::ActionType eAct);

private:
	// Utility functions
   template <typename PixelType>
   void TransposeSquareInPlace(PixelType* pI, unsigned int dim);
   template <typename PixelType>
   int TransposeRectangleOutOfPlace(PixelType* pI, unsigned int width, unsigned int height);

	// Data members
   bool busy_ = false;
	bool initialized_ = false;
   bool inPlace_ = false;
   void* pTemp_ = nullptr;
   unsigned long tempSize_ = 0;
};


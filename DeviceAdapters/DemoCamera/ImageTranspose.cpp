////////////////////////////////////////////////////////////////////////////////
// FILE:          ImageTranspose.cpp
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

#include "DemoHub.h"
#include "ImageTranspose.h"

extern const char* g_ImageTransposeName;

////////////////////////////////////////////////////////////////////////////////
// MM::Device API
////////////////////////////////////////////////////////////////////////////////

void ImageTranspose::GetName(char* name) const
{
   CDeviceUtils::CopyLimitedString(name, g_ImageTransposeName);
}

int ImageTranspose::Initialize()
{
   DemoHub* pHub = static_cast<DemoHub*>(GetParentHub());
   if (pHub)
   {
      char hubLabel[MM::MaxStrLength];
      pHub->GetLabel(hubLabel);
      SetParentID(hubLabel); // for backward comp.
   }
   else
      LogMessage(NoHubError);

   if( NULL != this->pTemp_)
   {
      free(pTemp_);
      pTemp_ = NULL;
      this->tempSize_ = 0;
   }
    CPropertyAction* pAct = new CPropertyAction (this, &ImageTranspose::OnInPlaceAlgorithm);
   (void)CreateIntegerProperty("InPlaceAlgorithm", 0, false, pAct);
   return DEVICE_OK;
}

int ImageTranspose::Shutdown()
{
   initialized_ = false;
   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// MM::ImageProcessor API
////////////////////////////////////////////////////////////////////////////////

int ImageTranspose::Process(unsigned char *pBuffer, unsigned int width, unsigned int height, unsigned int byteDepth)
{
   int ret = DEVICE_OK;
   if (width != height)
   {
      // problem with tranposing non-square images is that the image buffer
      // will need to be modified by the image processor.
      return DEVICE_NOT_SUPPORTED;
   }
   if (busy_)
   {
      return DEVICE_ERR;
   }
 
   busy_ = true;
   if (inPlace_)
   {
      if (sizeof(unsigned char) == byteDepth)
      {
         TransposeSquareInPlace((unsigned char*)pBuffer, width);
      }
      else if (sizeof(unsigned short) == byteDepth)
      {
         TransposeSquareInPlace((unsigned short*)pBuffer, width);
      }
      else if (sizeof(unsigned long) == byteDepth)
      {
         TransposeSquareInPlace((unsigned long*)pBuffer, width);
      }
      else if (sizeof(unsigned long long) == byteDepth)
      {
         TransposeSquareInPlace((unsigned long long*)pBuffer, width);
      }
      else 
      {
         ret = DEVICE_NOT_SUPPORTED;
      }
   }
   else
   {
      if (sizeof(unsigned char) == byteDepth)
      {
         ret = TransposeRectangleOutOfPlace((unsigned char*)pBuffer, width, height);
      }
      else if (sizeof(unsigned short) == byteDepth)
      {
         ret = TransposeRectangleOutOfPlace((unsigned short*)pBuffer, width, height);
      }
      else if (sizeof(unsigned long) == byteDepth)
      {
         ret = TransposeRectangleOutOfPlace((unsigned long*)pBuffer, width, height);
      }
      else if (sizeof(unsigned long long) == byteDepth)
      {
         ret =  TransposeRectangleOutOfPlace((unsigned long long*)pBuffer, width, height);
      }
      else
      {
         ret =  DEVICE_NOT_SUPPORTED;
      }
   }
   busy_ = false;

   return ret;
}

////////////////////////////////////////////////////////////////////////////////
// MM Action handlers
////////////////////////////////////////////////////////////////////////////////

int ImageTranspose::OnInPlaceAlgorithm(MM::PropertyBase* pProp, MM::ActionType eAct)
{
   if (eAct == MM::BeforeGet)
   {
      pProp->Set(this->inPlace_?1L:0L);
   }
   else if (eAct == MM::AfterSet)
   {
      long ltmp;
      pProp->Get(ltmp);
      inPlace_ = (0==ltmp?false:true);
   }

   return DEVICE_OK;
}

////////////////////////////////////////////////////////////////////////////////
// Utility functions
////////////////////////////////////////////////////////////////////////////////

template <typename PixelType>
void ImageTranspose::TransposeSquareInPlace(PixelType* pI, unsigned int dim)
{ 
   PixelType tmp;
   for( unsigned long ix = 0; ix < dim; ++ix)
   {
      for( unsigned long iy = ix; iy < dim; ++iy)
      {
         tmp = pI[iy * dim + ix];
         pI[iy * dim + ix] = pI[ix * dim + iy];
         pI[ix * dim + iy] = tmp; 
      }
   }

   return;
}

 // really primative image transpose algorithm which will work fine for non-square images... 
template <typename PixelType>
int TransposeRectangleOutOfPlace(PixelType* pI, unsigned int width, unsigned int height)
{
   int ret = DEVICE_OK;
   unsigned long tsize = width*height*sizeof(PixelType);
   if( this->tempSize_ != tsize)
   {
      if( NULL != this->pTemp_)
      {
         free(pTemp_);
         pTemp_ = NULL;
      }
      pTemp_ = (PixelType *)malloc(tsize);
   }
   if( NULL != pTemp_)
   {
      PixelType* pTmpImage = (PixelType *) pTemp_;
      tempSize_ = tsize;
      for( unsigned long ix = 0; ix < width; ++ix)
      {
         for( unsigned long iy = 0; iy < height; ++iy)
         {
            pTmpImage[iy + ix*width] = pI[ ix + iy*height];
         }
      }
      memcpy( pI, pTmpImage, tsize);
   }
   else
   {
      ret = DEVICE_ERR;
   }
   return ret;
}

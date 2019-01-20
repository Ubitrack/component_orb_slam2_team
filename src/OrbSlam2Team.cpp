/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2019, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

 /**
  * @ingroup vision_components
  * @file
  * @author Joe Bedard <bedard@in.tum.de>
  */

#include <string>
#include <list>
#include <iostream>
#include <iomanip>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>
#include <boost/numeric/ublas/blas.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>
#include <utUtil/BlockTimer.h>
#include <opencv/cv.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
#include <utAlgorithm/Homography.h>
#include <utAlgorithm/PoseEstimation2D3D/PlanarPoseEstimation.h>
#include <utAlgorithm/Projection.h>
#include <boost/numeric/ublas/blas.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

// ORB_SLAM2_TEAM includes
#include <Enums.h>
#include <ORBVocabulary.h>
#include <MapperServer.h>
#include <Tracking.h>
#include <FrameDrawer.h>

  //#include <RotationHelpers.h>

using namespace Ubitrack;
using namespace Ubitrack::Vision;

namespace Ubitrack {
   namespace Components {

      using namespace ORB_SLAM2_TEAM;

      // get a logger
      static log4cpp::Category& logger(log4cpp::Category::getInstance("Ubitrack.Vision.OrbSlam2TeamStereo"));

      /**
       * @ingroup vision_components
       *
       * @par Input Ports
       * None.
       *
       * @par Output Ports
       * \c Output push port of type Ubitrack::Measurement::ImageMeasurement.
       *
       */
      class OrbSlam2TeamStereo
         : public Dataflow::TriggerComponent
      {
      public:

         /** constructor */
         OrbSlam2TeamStereo(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >);

         /** destructor, waits until thread stops */
         ~OrbSlam2TeamStereo();

         /** starts the camera */
         void start();

         /** stops the camera */
         void stop();

         void compute(Measurement::Timestamp t);

      protected:

         // the ports
         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageL;
         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageR;
         //Dataflow::PullConsumer< Measurement::Matrix3x3 > m_inIntrinsics;
         //Dataflow::PushConsumer< Measurement::Button> m_eventIn;
         //Dataflow::PullConsumer<Measurement::Pose> m_pullCameraPose;

         Dataflow::PushSupplier< Measurement::ImageMeasurement > m_pushImgDebugL;
         Dataflow::TriggerOutPort< Measurement::Pose > m_outPose;
         Dataflow::PushSupplier< Measurement::ErrorPose > m_pushErrorPose;

      private:

         Util::BlockTimer m_timerTracking;
         Util::BlockTimer m_timerAll;
         bool m_LastDetection_success;
         bool m_isTracking;

         Math::Vector3d m_head2Eye;
         int m_initCount = 0;

         string m_debugWindowName;
         int m_maxDelay;
         int m_imageHeight;
         int m_imageWidth;

         // additional covariance
         double m_addErrorX;
         double m_addErrorY;
         double m_addErrorZ;

         // for debugging - prints the image format of the given image
         void printPixelFormat(Measurement::ImageMeasurement image);

         static ORBVocabulary * m_vocab;
         static Mapper * m_mapper;
         Tracking * m_tracker;
         FrameDrawer * m_frameDrawer;

      };

      ORBVocabulary * OrbSlam2TeamStereo::m_vocab = NULL;
      Mapper * OrbSlam2TeamStereo::m_mapper = NULL;

      OrbSlam2TeamStereo::OrbSlam2TeamStereo(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : Dataflow::TriggerComponent(sName, subgraph)
         , m_inImageL("ImageInputL", *this)
         , m_inImageR("ImageInputR", *this)
         //, m_inIntrinsics("Intrinsics", *this)
         //, m_eventIn("EventIn", *this, boost::bind(&OrbSlam2TeamStereo::buttonEvent, this, _1))
         //, m_pullCameraPose("CameraPose", *this)
         , m_pushImgDebugL("ImageDebugL", *this)
         , m_outPose("Output", *this)
         , m_pushErrorPose("OutputError", *this)
         , m_timerTracking("OrbSlam2TeamStereo.Tracking", logger)
         , m_timerAll("OrbSlam2TeamStereo.All", logger)
         , m_LastDetection_success(false)
         , m_isTracking(false)
         , m_maxDelay(30)
         , m_addErrorX(0.0)
         , m_addErrorY(0.0)
         , m_addErrorZ(0.0)
         , m_tracker(NULL)
      {
         subgraph->m_DataflowAttributes.getAttributeData("maxDelay", m_maxDelay);
         subgraph->m_DataflowAttributes.getAttributeData("debugWindowName", m_debugWindowName);

         std::string settingsFileName;
         if (subgraph->m_DataflowAttributes.hasAttribute("settingsFile"))
         {
            settingsFileName = subgraph->m_DataflowAttributes.getAttributeString("settingsFile");
            LOG4CPP_INFO(logger, "Settings File: " << settingsFileName);
         }
         else
         {
            std::ostringstream os;
            os << "ORB-SLAM2-TEAM Settings File is required, but was not provided!";
            UBITRACK_THROW(os.str());
         }

         std::string vocabularyFileName;
         if (subgraph->m_DataflowAttributes.hasAttribute("vocabularyFile"))
         {
            vocabularyFileName = subgraph->m_DataflowAttributes.getAttributeString("vocabularyFile");
            LOG4CPP_INFO(logger, "Vocabulary File: " << vocabularyFileName);
         }
         else
         {
            std::ostringstream os;
            os << "ORB-SLAM2-TEAM Vocabulary File is required, but was not provided!";
            UBITRACK_THROW(os.str());
         }

         if (m_vocab == NULL)
         {
            m_vocab = new ORBVocabulary();
            m_vocab->loadFromFile(vocabularyFileName);
         }

         if (m_mapper == NULL)
         {
            m_mapper = new MapperServer(*m_vocab, false, 2);
         }

         cv::FileStorage settings(settingsFileName, cv::FileStorage::READ);
         if (m_pushImgDebugL.isConnected())
         {
            m_frameDrawer = new FrameDrawer(settings);
            m_tracker = new Tracking(settings, *m_vocab, *m_mapper, m_frameDrawer, NULL, SensorType::STEREO);
         }
         else
         {
            m_frameDrawer = NULL;
            m_tracker = new Tracking(settings, *m_vocab, *m_mapper, NULL, NULL, SensorType::STEREO);
         }
      }

      OrbSlam2TeamStereo::~OrbSlam2TeamStereo()
      {
         delete m_tracker;
         delete m_frameDrawer;
      }

      void OrbSlam2TeamStereo::printPixelFormat(Measurement::ImageMeasurement image)
      {
         using Ubitrack::Vision::Image;
         switch (image->pixelFormat())
         {
         case Image::LUMINANCE:
            LOG4CPP_INFO(logger, "Image::LUMINANCE");
            break;
         case Image::RGB:
            LOG4CPP_INFO(logger, "Image::RGB");
            break;
         case Image::BGR:
            LOG4CPP_INFO(logger, "Image::BGR");
            break;
         case Image::RGBA:
            LOG4CPP_INFO(logger, "Image::RGBA");
            break;
         case Image::BGRA:
            LOG4CPP_INFO(logger, "Image::BGRA");
            break;
         case Image::YUV422:
            LOG4CPP_INFO(logger, "Image::YUV422");
            break;
         case Image::YUV411:
            LOG4CPP_INFO(logger, "Image::YUV411");
            break;
         case Image::RAW:
            LOG4CPP_INFO(logger, "Image::RAW");
            break;
         case Image::DEPTH:
            LOG4CPP_INFO(logger, "Image::DEPTH");
            break;
         case Image::UNKNOWN_PIXELFORMAT:
            LOG4CPP_INFO(logger, "Image::UNKNOWN_PIXELFORMAT");
            break;
         default:
            break;
         }
      }

      void OrbSlam2TeamStereo::compute(Measurement::Timestamp t)
      {
         UBITRACK_TIME(m_timerAll);

         Measurement::ImageMeasurement inImageL = m_inImageL.get();
         Measurement::ImageMeasurement inImageR = m_inImageR.get();
         printPixelFormat(inImageL);

         cv::Mat matImageL, matImageR;
         if (inImageL->origin() == 0) {
            matImageL = inImageL->Mat();
         }
         else {
            // the input image is flipped vertically
            cv::flip(inImageL->Mat(), matImageL, 0);
            //LOG4CPP_WARN(logger, "Left input image is flipped. Consider flipping in the driver to improve performance.");
         }
         if (inImageR->origin() == 0) {
            matImageR = inImageR->Mat();
         }
         else {
            // the input image is flipped vertically
            cv::flip(inImageR->Mat(), matImageR, 0);
            //LOG4CPP_WARN(logger, "Right input image is flipped. Consider flipping in the driver to improve performance.");
         }

         Measurement::Timestamp before;
         Measurement::Timestamp after;
         cv::Mat trackerPose;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            trackerPose = m_tracker->GrabImageStereo(matImageL, matImageR, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_maxDelay?

         Math::Matrix<double, 0Ui64, 0Ui64> mathMat = Math::Matrix<double, 0Ui64, 0Ui64>(4, 4);
         if (!trackerPose.empty())
         {
            for (short i = 0; i < 4; i++)
               for (short j = 0; j < 4; j++)
                  mathMat.at_element(0, 0) = trackerPose.at<double>(i, j);
         }
         Math::Pose mathPose = Math::Pose(mathMat);
         Measurement::Pose measurementPose = Measurement::Pose(inImageL.time(), mathPose);
         m_outPose.send(measurementPose);


         cv::imshow("m_debugWindowName", m_frameDrawer->DrawFrame());
         if (!m_debugWindowName.empty())
         {
            cv::imshow(m_debugWindowName, m_frameDrawer->DrawFrame());
         }

         if (m_pushImgDebugL.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebugL.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }


      void OrbSlam2TeamStereo::start()
      {
         Component::start();
      }


      void OrbSlam2TeamStereo::stop()
      {
         Component::stop();
      }

   }
} // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT(Dataflow::ComponentFactory* const cf) {
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamStereo >("OrbSlam2TeamStereo");
}


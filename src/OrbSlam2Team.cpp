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

         Dataflow::PushSupplier< Measurement::ImageMeasurement > m_pushImgDebugL;
         Dataflow::TriggerOutPort< Measurement::Pose > m_outPose;
         Dataflow::PushSupplier< Measurement::ErrorPose > m_pushErrorPose;

      private:

         Util::BlockTimer m_timerTracking;
         Util::BlockTimer m_timerAll;
         string m_settingsFileName;

         int m_maxDelay;

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
         , m_pushImgDebugL("ImageDebugL", *this)
         , m_outPose("Output", *this)
         , m_pushErrorPose("OutputError", *this)
         , m_timerTracking("OrbSlam2TeamStereo.Tracking", logger)
         , m_timerAll("OrbSlam2TeamStereo.All", logger)
         , m_maxDelay(30)
         , m_addErrorX(0.0)
         , m_addErrorY(0.0)
         , m_addErrorZ(0.0)
         , m_tracker(NULL)
      {
         subgraph->m_DataflowAttributes.getAttributeData("maxDelay", m_maxDelay);

         if (subgraph->m_DataflowAttributes.hasAttribute("settingsFile"))
         {
            m_settingsFileName = subgraph->m_DataflowAttributes.getAttributeString("settingsFile");
            LOG4CPP_INFO(logger, "Settings File: " << m_settingsFileName);
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
            LOG4CPP_DEBUG(logger, "Image::LUMINANCE");
            break;
         case Image::RGB:
            LOG4CPP_DEBUG(logger, "Image::RGB");
            break;
         case Image::BGR:
            LOG4CPP_DEBUG(logger, "Image::BGR");
            break;
         case Image::RGBA:
            LOG4CPP_DEBUG(logger, "Image::RGBA");
            break;
         case Image::BGRA:
            LOG4CPP_DEBUG(logger, "Image::BGRA");
            break;
         case Image::YUV422:
            LOG4CPP_DEBUG(logger, "Image::YUV422");
            break;
         case Image::YUV411:
            LOG4CPP_DEBUG(logger, "Image::YUV411");
            break;
         case Image::RAW:
            LOG4CPP_DEBUG(logger, "Image::RAW");
            break;
         case Image::DEPTH:
            LOG4CPP_DEBUG(logger, "Image::DEPTH");
            break;
         case Image::UNKNOWN_PIXELFORMAT:
            LOG4CPP_DEBUG(logger, "Image::UNKNOWN_PIXELFORMAT");
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

         if (m_pushImgDebugL.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebugL.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }


      void OrbSlam2TeamStereo::start()
      {
         Component::start();

         cv::FileStorage settings(m_settingsFileName, cv::FileStorage::READ);
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


      void OrbSlam2TeamStereo::stop()
      {
         Component::stop();
      }

   }
} // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT(Dataflow::ComponentFactory* const cf) {
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamStereo >("OrbSlam2TeamStereo");
}


//#include <utDataflow/Module.h>
//
//// Module Key
//MAKE_NODEATTRIBUTEKEY_DEFAULT( MapperKey, int, "Origin", "mapperId", 1 );
//
//// Component Key
//class TrackerKey
//{
//public:
//   TrackerKey(boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
//   {
//      Graph::UTQLSubgraph::EdgePtr config;
//
//      if ( subgraph->hasEdge( "Output" ) )
//         config = subgraph->getEdge( "Output" );
//
//      if ( !config )
//      {
//         UBITRACK_THROW( "OrbSlam2Team Pattern is missing \"Output\" edge");
//      }
//
//      config->getAttributeData( "artBodyId", m_body );
//      if ( m_body <= 0 )
//         UBITRACK_THROW( "Missing or invalid \"artBodyId\" attribute on \"ArtToTarget\" resp. \"fingerHandOutput\" edge" );
//
//      std::string typeString = config->getAttributeString( "artType" );
//      if ( typeString.empty() )
//      {
//         // no explicit art target type information. so we assume 6D
//         m_targetType = target_6d;
//      }
//      else
//      {
//         if ( typeString == "6d" )
//            m_targetType = target_6d;
//         else if ( typeString == "6df" )
//            m_targetType = target_6d_flystick;
//         else if ( typeString == "6dmt" )
//            m_targetType = target_6d_measurement_tool;
//         else if ( typeString == "6dmtr" )
//            m_targetType = target_6d_measurement_tool_reference;
//         else if ( typeString == "3dcloud" )
//         {
//            m_targetType = target_3dcloud;
//            m_body = 0;
//         }
//         else if ( typeString == "finger" )
//         {
//            m_targetType = target_finger;
//
//            /*
//            Graph::UTQLSubgraph::NodePtr configNode = config->m_Target.lock();
//
//            std::string fingerString = configNode->getAttributeString( "finger" );
//
//            if (fingerString.length() == 0)
//            UBITRACK_THROW( "Art finger target without finger id" );
//
//            if ( fingerString == "hand" )
//            m_fingerType = finger_hand;
//            else if ( fingerString == "thumb" )
//            m_fingerType = finger_thumb;
//            else if ( fingerString == "index" )
//            m_fingerType = finger_index;
//            else if ( fingerString == "middle" )
//            m_fingerType = finger_middle;
//            else
//            UBITRACK_THROW( "Art finger target with unknown finger type: " + fingerString );
//            */
//
//            std::string fingerSideString = config->getAttributeString( "fingerSide" );
//            if (fingerSideString.length() == 0)
//               UBITRACK_THROW( "Art finger target without finger side" );
//
//            if ( fingerSideString == "left" )
//               m_fingerSide = side_left;
//            else if ( fingerSideString == "right" )
//               m_fingerSide = side_right;
//            else
//               UBITRACK_THROW( "Art finger target with unknown finger side: " + fingerSideString );
//
//         }
//         else
//            UBITRACK_THROW( "Art target with unknown target type: " + typeString );
//      }
//   }
//};
//
//class OrbSlam2TeamTracker; // Component
//
//class OrbSlam2TeamMapper
//   : public Dataflow::Module< MapperKey, TrackerKey, OrbSlam2TeamMapper, OrbSlam2TeamTracker >
//{
//public:
//
//   // override factory method to create components
//   // see: component_vision::MarkerTracker
//   boost::shared_ptr< MarkerTrackerBase > createComponent( const std::string& type, const std::string& name,
//      boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const ComponentKey& key, ModuleClass* pModule );
//};
//
//class OrbSlam2TeamTracker
//   : public OrbSlam2TeamMapper::Component
//{
//
//};

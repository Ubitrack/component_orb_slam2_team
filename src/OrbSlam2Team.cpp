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

#include <utDataflow/Module.h>
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
      static log4cpp::Category& logger(log4cpp::Category::getInstance("Ubitrack.Vision.OrbSlam2Team"));

      static Math::Pose CvMatPoseToMathPose(cv::Mat & m)
      {
         Math::Matrix<double, 0Ui64, 0Ui64> mathMat = Math::Matrix<double, 0Ui64, 0Ui64>(4, 4);
         if (!m.empty())
         {
            for (short i = 0; i < 4; i++)
               for (short j = 0; j < 4; j++)
                  mathMat.at_element(i, j) = m.at<double>(i, j);
         }
         return Math::Pose(mathMat);
      }

      class OrbSlam2TeamComponent;


      // Module Key
      MAKE_NODEATTRIBUTEKEY_DEFAULT( MapperKey, int, "Mapper", "mapperId", 1 );


      // Component Key
      class TrackerKey
      {
      public:
         TrackerKey(boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         {
            // the subgraph should be one of the OrbSlam2Team patterns
            // the ID should be a unique string such as "pattern_19"
            m_id = subgraph->m_ID;

            if (m_id.empty())
            {
               ostringstream s;
               s << "Missing or invalid \"id\" attribute on " << subgraph->m_Name << " pattern";
               UBITRACK_THROW(s.str());
            }
         }

         // less than operator for map
         bool operator<(const TrackerKey & rVal) const
         {
            return m_id < rVal.m_id;
         }

      private:
         string m_id;
      };


      class OrbSlam2TeamModule
         : public Dataflow::Module< MapperKey, TrackerKey, OrbSlam2TeamModule, OrbSlam2TeamComponent >
      {
      public:

         /** UTQL constructor */
         OrbSlam2TeamModule( const MapperKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory );

         virtual void startModule();

         virtual void stopModule();

         // override factory method to create components
         boost::shared_ptr<OrbSlam2TeamComponent> createComponent( const string& type, const string& name,
            boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const TrackerKey& key, OrbSlam2TeamModule* pModule );

         ORBVocabulary * m_vocab;

         MapperServer * m_mapper;

      private:
         string m_vocabularyFilePath;
         bool m_mono;
         unsigned int m_maxTrackers;
      };

      // can't use the module-component technique until Module supports TriggerComponent
      class OrbSlam2TeamComponent
         : public OrbSlam2TeamModule::Component
      {
      public:
         OrbSlam2TeamComponent(const string& name, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const TrackerKey& componentKey, OrbSlam2TeamModule* module)
            : OrbSlam2TeamModule::Component(name, componentKey, module)
         {
            if (module)
            {
               m_vocab = module->m_vocab;
               m_mapper = module->m_mapper;
            }
            else
               UBITRACK_THROW("OrbSlam2TeamComponent::OrbSlam2TeamComponent : module is null");
         }

      //protected:
         ORBVocabulary * m_vocab;
         Mapper * m_mapper;
      };


      /**
      * @ingroup vision_components
      *
      * @par Input Ports
      * \c Ubitrack::Measurement::ImageMeasurement ImageInputL
      * \c Ubitrack::Measurement::ImageMeasurement ImageInputR
      *
      * @par Output Ports
      * \c Ubitrack::Measurement::Pose Output
      * \c Ubitrack::Measurement::ErrorPose OutputError 
      */
      class OrbSlam2TeamStereo
         : public Dataflow::TriggerComponent
      {
      public:

         OrbSlam2TeamStereo(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph);

         virtual void start();

         virtual void stop();

         void compute(Measurement::Timestamp t);

      protected:

         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageL;
         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageR;
         Dataflow::PushSupplier< Measurement::ImageMeasurement > m_pushImgDebugL;
         Dataflow::TriggerOutPort< Measurement::Pose > m_outPose;
         Dataflow::PushSupplier< Measurement::ErrorPose > m_pushErrorPose;
         Dataflow::PushSupplier< Measurement::Pose > m_outBaseline;

      private:

         Util::BlockTimer m_timerTracking;
         Util::BlockTimer m_timerAll;
         int m_maxDelay;
         
         // additional covariance
         double m_addErrorX;
         double m_addErrorY;
         double m_addErrorZ;

         // for debugging - prints the image format of the given image
         void printPixelFormat(Measurement::ImageMeasurement image);

         string m_settingsFileName;
         Tracking * m_tracker;
         FrameDrawer * m_frameDrawer;

         static boost::shared_ptr<ORBVocabulary> m_vocab;
         static boost::shared_ptr<Mapper> m_mapper;
         static unsigned int m_maxTrackers;
      };

      boost::shared_ptr<ORBVocabulary> OrbSlam2TeamStereo::m_vocab = boost::shared_ptr<ORBVocabulary>(NULL);
      boost::shared_ptr<Mapper> OrbSlam2TeamStereo::m_mapper = boost::shared_ptr<Mapper>(NULL);
      unsigned int OrbSlam2TeamStereo::m_maxTrackers = 0;

      OrbSlam2TeamStereo::OrbSlam2TeamStereo(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : Dataflow::TriggerComponent(sName, subgraph)
         , m_inImageL("ImageInputL", *this)
         , m_inImageR("ImageInputR", *this)
         , m_pushImgDebugL("ImageDebugL", *this)
         , m_outPose("Output", *this)
         , m_outBaseline("Baseline", *this)
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
            ostringstream os;
            os << "ORB-SLAM2-TEAM Settings File is required, but was not provided!";
            UBITRACK_THROW(os.str());
         }

         if (!m_mapper)
         {
            Graph::UTQLSubgraph::NodePtr nodeMapper;
            if (subgraph->hasNode("Mapper"))
               nodeMapper = subgraph->getNode("Mapper");
            if (!nodeMapper)
               UBITRACK_THROW( "OrbSlam2Team Pattern is missing \"Mapper\" node");

            string vocabularyFilePath;
            if (nodeMapper->hasAttribute("vocabularyFile"))
            {
               nodeMapper->getAttributeData("vocabularyFile", vocabularyFilePath);
               LOG4CPP_INFO(logger, "Vocabulary File: " << vocabularyFilePath);
               if (vocabularyFilePath.empty())
               {
                  ostringstream os;
                  os << "ORB-SLAM2-TEAM Vocabulary File is required, but was not provided!";
                  UBITRACK_THROW(os.str());
               }
            }
            else
            {
               UBITRACK_THROW( "Missing or invalid \"vocabularyFile\" attribute on \"Mapper\" node" );
            }

            m_vocab = boost::shared_ptr<ORBVocabulary>(new ORBVocabulary());
            m_vocab->loadFromFile(vocabularyFilePath);

            if (nodeMapper->hasAttribute("maxTrackers"))
            {
               nodeMapper->getAttributeData("maxTrackers", m_maxTrackers);
               LOG4CPP_INFO(logger, "Maximum Trackers: " << m_maxTrackers);
               if (m_maxTrackers < 1)
               {
                  ostringstream os;
                  os << "Maximum Trackers must be at least 1!";
                  UBITRACK_THROW(os.str());
               }
            }
            else
            {
               UBITRACK_THROW( "Missing or invalid \"maxTrackers\" attribute on \"Mapper\" node" );
            }

            m_mapper = boost::shared_ptr<Mapper>(new MapperServer(*m_vocab, false, m_maxTrackers));
         }

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

         Math::Pose mathPose = CvMatPoseToMathPose(trackerPose);
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
         TriggerComponent::start();

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

         Math::Pose mathPose = CvMatPoseToMathPose(m_tracker->GetBaseline());
         Measurement::Pose measurementPose = Measurement::Pose(Measurement::Timestamp(), mathPose);
         m_outBaseline.send(measurementPose);
      }
      
      void OrbSlam2TeamStereo::stop()
      {
         TriggerComponent::stop();

         delete m_tracker;
         delete m_frameDrawer;
      }


      boost::shared_ptr<OrbSlam2TeamComponent> OrbSlam2TeamModule::createComponent(const string& type, const string& name,
         boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const TrackerKey& key, OrbSlam2TeamModule* pModule)
      {
         LOG4CPP_INFO(logger, "Class " + type + " created by OrbSlam2TeamModule " );
         //if ( type == "OrbSlam2TeamStereo" )
         //   return boost::shared_ptr< OrbSlam2TeamComponent >(new OrbSlam2TeamStereo( name, subgraph, key, pModule ));
         //else if ( type == "OrbSlam2TeamMono" )
         //   return boost::shared_ptr< OrbSlam2TeamComponent >(new OrbSlam2TeamMono( name, subgraph, key, module ));

         UBITRACK_THROW( "Class " + type + " not supported by OrbSlam2TeamModule" );
      }

      OrbSlam2TeamModule::OrbSlam2TeamModule(const MapperKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory)
         : Module< MapperKey, TrackerKey, OrbSlam2TeamModule, OrbSlam2TeamComponent >( key, pFactory )
         , m_vocab(NULL)
         , m_mapper(NULL)
      {
         Graph::UTQLSubgraph::NodePtr nodeMapper;
         if (subgraph->hasNode("Mapper"))
            nodeMapper = subgraph->getNode("Mapper");
         if (!nodeMapper)
            UBITRACK_THROW( "OrbSlam2Team Pattern is missing \"Mapper\" node");

         if ("OrbSlam2TeamStereo" == subgraph->m_DataflowClass)
            m_mono = false;
         else if ("OrbSlam2TeamMono" == subgraph->m_DataflowClass)
            m_mono = true;
         else
         {
            ostringstream s;
            s << subgraph->m_Name << " pattern with unhandled " << subgraph->m_DataflowClass << " class in OrbSlam2TeamModule";
            UBITRACK_THROW(s.str());
         }

         if (nodeMapper->hasAttribute("vocabularyFile"))
         {
            nodeMapper->getAttributeData("vocabularyFile", m_vocabularyFilePath);
            LOG4CPP_INFO(logger, "Vocabulary File: " << m_vocabularyFilePath);
            if (m_vocabularyFilePath.empty())
            {
               ostringstream os;
               os << "ORB-SLAM2-TEAM Vocabulary File is required, but was not provided!";
               UBITRACK_THROW(os.str());
            }
         }
         else
         {
            UBITRACK_THROW( "Missing or invalid \"vocabularyFile\" attribute on \"Mapper\" node" );
         }

         if (nodeMapper->hasAttribute("maxTrackers"))
         {
            nodeMapper->getAttributeData("maxTrackers", m_maxTrackers);
            LOG4CPP_INFO(logger, "Maximum Trackers: " << m_maxTrackers);
            if (m_maxTrackers < 1)
            {
               ostringstream os;
               os << "Maximum Trackers must be at least 1!";
               UBITRACK_THROW(os.str());
            }
         }
         else
         {
            UBITRACK_THROW( "Missing or invalid \"maxTrackers\" attribute on \"Mapper\" node" );
         }
      }

      void OrbSlam2TeamModule::startModule()
      {
         m_vocab = new ORBVocabulary();
         m_vocab->loadFromFile(m_vocabularyFilePath);
         m_mapper = new MapperServer(*m_vocab, m_mono, m_maxTrackers);
      }

      void OrbSlam2TeamModule::stopModule()
      {
         delete m_mapper;
         delete m_vocab;
      }

   } // namespace Components
} // namespace Ubitrack

UBITRACK_REGISTER_COMPONENT(Dataflow::ComponentFactory * const cf) {
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamStereo >("OrbSlam2TeamStereo");
   //cf->registerModule< Ubitrack::Components::OrbSlam2TeamModule >("OrbSlam2TeamStereo");
}



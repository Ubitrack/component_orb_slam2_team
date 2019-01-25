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

#include "OrbSlam2Team.h"

// ORB_SLAM2_TEAM includes
#include <Enums.h>

namespace Ubitrack {
   namespace Components {

      using namespace Ubitrack;
      using namespace ORB_SLAM2_TEAM;

      // get a logger
      static log4cpp::Category& logger(log4cpp::Category::getInstance("Ubitrack.Component.Vision.OrbSlam2Team"));

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

      boost::shared_ptr<ORBVocabulary> OrbSlam2TeamBase::m_vocab = boost::shared_ptr<ORBVocabulary>(NULL);
      boost::shared_ptr<Mapper> OrbSlam2TeamBase::m_mapper = boost::shared_ptr<Mapper>(NULL);
      unsigned int OrbSlam2TeamBase::m_maxTrackers = 0;

      OrbSlam2TeamBase::OrbSlam2TeamBase(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, SensorType sensor)
         : Dataflow::TriggerComponent(sName, subgraph)
         , m_sensor(sensor)
         , m_pushImgDebug("ImageDebug", *this)
         , m_outPose("Output", *this)
         , m_pushErrorPose("OutputError", *this)
         , m_pullMapPoints("MapPoints", *this, boost::bind(&OrbSlam2TeamBase::pullMapPoints, this, _1))
         , m_pullKeyFrames("KeyFrames", *this, boost::bind(&OrbSlam2TeamBase::pullKeyFrames, this, _1))
         , m_timerTracking("OrbSlam2TeamBase.Tracking", logger)
         , m_timerAll("OrbSlam2TeamBase.All", logger)
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
               UBITRACK_THROW("OrbSlam2Team Pattern is missing \"Mapper\" node");

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
               UBITRACK_THROW("Missing or invalid \"vocabularyFile\" attribute on \"Mapper\" node");
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
               UBITRACK_THROW("Missing or invalid \"maxTrackers\" attribute on \"Mapper\" node");
            }

            m_mapper = boost::shared_ptr<Mapper>(new MapperServer(*m_vocab, sensor == SensorType::MONOCULAR, m_maxTrackers));
         }
      }

      OrbSlam2TeamStereo::OrbSlam2TeamStereo(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamBase(sName, subgraph, SensorType::STEREO)
         , m_inImageL("ImageInputL", *this)
         , m_inImageR("ImageInputR", *this)
         , m_outBaseline("Baseline", *this)
      {

      }

      OrbSlam2TeamMono::OrbSlam2TeamMono(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamBase(sName, subgraph, SensorType::MONOCULAR)
         , m_inImage("ImageInput", *this)
      {

      }

      OrbSlam2TeamRgbd::OrbSlam2TeamRgbd(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamBase(sName, subgraph, SensorType::RGBD)
         , m_inImageRgb("ImageInput", *this)
         , m_inImageD("ImageInputD", *this)
         , m_outBaseline("Baseline", *this)
      {
         
      }

      void OrbSlam2TeamBase::printPixelFormat(Measurement::ImageMeasurement image)
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

      Measurement::PositionList OrbSlam2TeamBase::pullMapPoints(Ubitrack::Measurement::Timestamp t)
      {
         Measurement::PositionList posList(t);
         for (MapPoint * pMP : m_mapper->GetMap().GetAllMapPoints())
         {
            cv::Mat wPos = pMP->GetWorldPos();
            Math::Vector3d v3d(wPos.at<float>(0), wPos.at<float>(1), wPos.at<float>(2));
            posList->push_back(v3d);
         }
         return posList;
      }

      Measurement::PoseList OrbSlam2TeamBase::pullKeyFrames(Ubitrack::Measurement::Timestamp t)
      {
         Measurement::PoseList posList(t);
         for (KeyFrame * pKF : m_mapper->GetMap().GetAllKeyFrames())
         {
            Math::Pose pose = CvMatPoseToMathPose(pKF->GetPose());
            posList->push_back(pose);
         }
         return posList;
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

         if (m_pushImgDebug.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebug.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }
      
      void OrbSlam2TeamMono::compute(Measurement::Timestamp t)
      {
         UBITRACK_TIME(m_timerAll);

         Measurement::ImageMeasurement inImage = m_inImage.get();
         printPixelFormat(inImage);

         cv::Mat matImage;
         if (inImage->origin() == 0) {
            matImage = inImage->Mat();
         }
         else {
            // the input image is flipped vertically
            cv::flip(inImage->Mat(), matImage, 0);
            //LOG4CPP_WARN(logger, "Input image is flipped. Consider flipping in the driver to improve performance.");
         }

         Measurement::Timestamp before;
         Measurement::Timestamp after;
         cv::Mat trackerPose;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            trackerPose = m_tracker->GrabImageMonocular(matImage, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_maxDelay?

         Math::Pose mathPose = CvMatPoseToMathPose(trackerPose);
         Measurement::Pose measurementPose = Measurement::Pose(inImage.time(), mathPose);
         m_outPose.send(measurementPose);

         if (m_pushImgDebug.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebug.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }

      void OrbSlam2TeamRgbd::compute(Measurement::Timestamp t)
      {
         UBITRACK_TIME(m_timerAll);

         Measurement::ImageMeasurement inImageRgb = m_inImageRgb.get();
         Measurement::ImageMeasurement inImageD = m_inImageD.get();
         printPixelFormat(inImageRgb);

         cv::Mat matImageRgb, matImageD;
         if (inImageRgb->origin() == 0) {
            matImageRgb = inImageRgb->Mat();
         }
         else {
            // the input image is flipped vertically
            cv::flip(inImageRgb->Mat(), matImageRgb, 0);
            //LOG4CPP_WARN(logger, "RGB input image is flipped. Consider flipping in the driver to improve performance.");
         }
         if (inImageD->origin() == 0) {
            matImageD = inImageD->Mat();
         }
         else {
            // the input image is flipped vertically
            cv::flip(inImageD->Mat(), matImageD, 0);
            //LOG4CPP_WARN(logger, "Depth input image is flipped. Consider flipping in the driver to improve performance.");
         }

         Measurement::Timestamp before;
         Measurement::Timestamp after;
         cv::Mat trackerPose;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            trackerPose = m_tracker->GrabImageRGBD(matImageRgb, matImageD, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_maxDelay?

         Math::Pose mathPose = CvMatPoseToMathPose(trackerPose);
         Measurement::Pose measurementPose = Measurement::Pose(inImageRgb.time(), mathPose);
         m_outPose.send(measurementPose);

         if (m_pushImgDebug.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebug.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }

      void OrbSlam2TeamBase::start()
      {
         TriggerComponent::start();

         cv::FileStorage settings(m_settingsFileName, cv::FileStorage::READ);
         if (m_pushImgDebug.isConnected())
         {
            m_frameDrawer = new FrameDrawer(settings);
            m_tracker = new Tracking(settings, *m_vocab, *m_mapper, m_frameDrawer, NULL, m_sensor);
         }
         else
         {
            m_frameDrawer = NULL;
            m_tracker = new Tracking(settings, *m_vocab, *m_mapper, NULL, NULL, m_sensor);
         }
      }

      void OrbSlam2TeamStereo::start()
      {
         OrbSlam2TeamBase::start();

         Math::Pose mathPose = CvMatPoseToMathPose(m_tracker->GetBaseline());
         Measurement::Pose measurementPose = Measurement::Pose(Measurement::Timestamp(), mathPose);
         m_outBaseline.send(measurementPose);
      }
      
      void OrbSlam2TeamRgbd::start()
      {
         OrbSlam2TeamBase::start();

         Math::Pose mathPose = CvMatPoseToMathPose(m_tracker->GetBaseline());
         Measurement::Pose measurementPose = Measurement::Pose(Measurement::Timestamp(), mathPose);
         m_outBaseline.send(measurementPose);
      }

      void OrbSlam2TeamBase::stop()
      {
         TriggerComponent::stop();

         delete m_tracker;
         delete m_frameDrawer;
      }


      boost::shared_ptr<OrbSlam2TeamComponent> OrbSlam2TeamModule::createComponent(const string& type, const string& name,
         boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const TrackerKey& key, OrbSlam2TeamModule* pModule)
      {
         LOG4CPP_INFO(logger, "Class " + type + " created by OrbSlam2TeamModule ");
         //if ( type == "OrbSlam2TeamStereo" )
         //   return boost::shared_ptr< OrbSlam2TeamComponent >(new OrbSlam2TeamStereo( name, subgraph, key, pModule ));
         //else if ( type == "OrbSlam2TeamMono" )
         //   return boost::shared_ptr< OrbSlam2TeamComponent >(new OrbSlam2TeamMono( name, subgraph, key, module ));

         UBITRACK_THROW("Class " + type + " not supported by OrbSlam2TeamModule");
      }

      OrbSlam2TeamModule::OrbSlam2TeamModule(const MapperKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory)
         : Module< MapperKey, TrackerKey, OrbSlam2TeamModule, OrbSlam2TeamComponent >(key, pFactory)
         , m_vocab(NULL)
         , m_mapper(NULL)
      {
         Graph::UTQLSubgraph::NodePtr nodeMapper;
         if (subgraph->hasNode("Mapper"))
            nodeMapper = subgraph->getNode("Mapper");
         if (!nodeMapper)
            UBITRACK_THROW("OrbSlam2Team Pattern is missing \"Mapper\" node");

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
            UBITRACK_THROW("Missing or invalid \"vocabularyFile\" attribute on \"Mapper\" node");
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
            UBITRACK_THROW("Missing or invalid \"maxTrackers\" attribute on \"Mapper\" node");
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


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
      
      static Math::Matrix3x3d CvMatfToMatrix3x3d(cv::Mat & m)
      {
         Math::Matrix3x3d mathMat;
         if (!m.empty())
         {
            for (short i = 0; i < 3; i++)
               for (short j = 0; j < 3; j++)
                  mathMat.at_element(i, j) = m.at<float>(i, j);
         }
         return mathMat;
      }

      boost::shared_ptr<ORBVocabulary> OrbSlam2TeamBase::m_vocab = boost::shared_ptr<ORBVocabulary>(NULL);
      boost::shared_ptr<Mapper> OrbSlam2TeamBase::m_mapper = boost::shared_ptr<Mapper>(NULL);
      unsigned int OrbSlam2TeamBase::m_maxTrackers = 0;

      OrbSlam2TeamBase::OrbSlam2TeamBase(boost::shared_ptr< Graph::UTQLSubgraph > subgraph) 
         : m_msDelay(30)
      {
         subgraph->m_DataflowAttributes.getAttributeData("msDelay", m_msDelay);
      }

      OrbSlam2TeamBase::OrbSlam2TeamBase(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, SensorType sensor)
         : m_sensor(sensor)
         , m_msDelay(30)
      {
         subgraph->m_DataflowAttributes.getAttributeData("msDelay", m_msDelay);

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

      OrbSlam2TeamRender::OrbSlam2TeamRender(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : Dataflow::Component(sName)
         , OrbSlam2TeamBase(subgraph)
         , m_pushMapPoints("MapPoints", *this)
         , m_pushKeyFrames("KeyFrames", *this)
         , m_run(true)
      {

      }

      void OrbSlam2TeamRender::start()
      {
         m_thread = new boost::thread(&OrbSlam2TeamRender::pushData, this);
         LOG4CPP_DEBUG(logger, "created push thread ms==" << m_msDelay);
      }

      void OrbSlam2TeamRender::stop()
      {
         m_run = false;
         m_thread->join();
         delete m_thread;
      }

      OrbSlam2TeamTracker::OrbSlam2TeamTracker(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, SensorType sensor)
         : Dataflow::TriggerComponent(sName, subgraph)
         , OrbSlam2TeamBase(sName, subgraph, sensor)
         , m_pushImgDebug("ImageDebug", *this)
         , m_outPose("Output", *this)
         , m_pushErrorPose("OutputError", *this)
         , m_timerTracking("OrbSlam2TeamTracker.Tracking", logger)
         , m_timerAll("OrbSlam2TeamTracker.All", logger)
         , m_addErrorX(0.0)
         , m_addErrorY(0.0)
         , m_addErrorZ(0.0)
         , m_tracker(NULL)
      {
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

      }

      OrbSlam2TeamStereo::OrbSlam2TeamStereo(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamTracker(sName, subgraph, SensorType::STEREO)
         , m_inImageL("ImageInputL", *this)
         , m_inImageR("ImageInputR", *this)
         , m_outBaseline("Baseline", *this)
      {

      }

      OrbSlam2TeamMono::OrbSlam2TeamMono(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamTracker(sName, subgraph, SensorType::MONOCULAR)
         , m_inImage("ImageInput", *this)
      {

      }

      OrbSlam2TeamRgbd::OrbSlam2TeamRgbd(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
         : OrbSlam2TeamTracker(sName, subgraph, SensorType::RGBD)
         , m_inImageRgb("ImageInput", *this)
         , m_inImageD("ImageInputD", *this)
         , m_outBaseline("Baseline", *this)
      {
         
      }

      void OrbSlam2TeamTracker::printPixelFormat(Measurement::ImageMeasurement image)
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

      void OrbSlam2TeamRender::pushData()
      {
         while (m_run)
         {
            Measurement::Timestamp start = Measurement::now();
            if (m_mapper)
            {

               if (m_pushMapPoints.isConnected())
               {
                  Measurement::PositionList posiList(start, vector<Math::Vector3d>());
                  for (MapPoint * pMP : m_mapper->GetMap().GetAllMapPoints())
                  {
                     cv::Mat wPos = pMP->GetWorldPos();
                     Math::Vector3d v3d(wPos.at<float>(0), wPos.at<float>(1), wPos.at<float>(2));
                     posiList->push_back(v3d);
                  }
                  LOG4CPP_DEBUG(logger, "Pushing Map Points size==" << posiList->size());
                  if (posiList->size() > 0)
                     m_pushMapPoints.send(posiList);
               }

               if (m_pushKeyFrames.isConnected())
               {
                  Measurement::PoseList poseList(start, vector<Math::Pose>());
                  for (KeyFrame * pKF : m_mapper->GetMap().GetAllKeyFrames())
                  {
                     Math::Pose pose = CvMatPoseToMathPose(pKF->GetPose());
                     poseList->push_back(pose);
                  }
                  LOG4CPP_DEBUG(logger, "Pushing Key Frames size==" << poseList->size());
                  m_pushKeyFrames.send(poseList);
               }
            }
            Measurement::Timestamp stop = Measurement::now();
            unsigned long duration = (stop - start) / 1000000l;
            if (duration < m_msDelay && m_run)
            {
               Util::sleep(m_msDelay - duration);
            }
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
         static Frame blank;
         Frame & f = blank;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            f = m_tracker->GrabImageStereo(matImageL, matImageR, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_msDelay?

         Math::Pose mathPose = CvMatPoseToMathPose(f.mTcw);
         Measurement::Pose measurementPose = Measurement::Pose(inImageL.time(), mathPose);
         m_outPose.send(measurementPose);

         if (m_pushErrorPose.isConnected())
         {
            sendErrorPose(t, f, mathPose);
         }

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
         static Frame blank;
         Frame & f = blank;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            f = m_tracker->GrabImageMonocular(matImage, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_msDelay?

         Math::Pose mathPose = CvMatPoseToMathPose(f.mTcw);
         Measurement::Pose measurementPose = Measurement::Pose(inImage.time(), mathPose);
         m_outPose.send(measurementPose);

         if (m_pushErrorPose.isConnected())
         {
            sendErrorPose(t, f, mathPose);
         }

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
         static Frame blank;
         Frame & f = blank;
         {
            UBITRACK_TIME(m_timerTracking);

            // pass the image to ORB-SLAM2-TEAM
            before = Measurement::now();
            f = m_tracker->GrabImageRGBD(matImageRgb, matImageD, t);
            after = Measurement::now();
         }
         Measurement::Timestamp diff = (after - before) / 1000000l;
         // do something with the time difference? m_msDelay?

         Math::Pose mathPose = CvMatPoseToMathPose(f.mTcw);
         Measurement::Pose measurementPose = Measurement::Pose(inImageRgb.time(), mathPose);
         m_outPose.send(measurementPose);

         if (m_pushErrorPose.isConnected())
         {
            sendErrorPose(t, f, mathPose);
         }

         if (m_pushImgDebug.isConnected())
         {
            Vision::Image img(m_frameDrawer->DrawFrame());
            m_pushImgDebug.send(Measurement::ImageMeasurement(t, img.Clone()));
         }
      }

      void OrbSlam2TeamTracker::start()
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
         OrbSlam2TeamTracker::start();

         Math::Pose mathPose = CvMatPoseToMathPose(m_tracker->GetBaseline());
         Measurement::Pose measurementPose = Measurement::Pose(Measurement::now(), mathPose);
         m_outBaseline.send(measurementPose);
      }
      
      void OrbSlam2TeamRgbd::start()
      {
         OrbSlam2TeamTracker::start();

         Math::Pose mathPose = CvMatPoseToMathPose(m_tracker->GetBaseline());
         Measurement::Pose measurementPose = Measurement::Pose(Measurement::now(), mathPose);
         m_outBaseline.send(measurementPose);
      }

      void OrbSlam2TeamTracker::stop()
      {
         TriggerComponent::stop();

         delete m_tracker;
         delete m_frameDrawer;
      }

      void OrbSlam2TeamTracker::sendErrorPose(Measurement::Timestamp & t, Frame & f, Math::Pose & mathPose)
      {
         cv::Mat matIntrinsics = f.mFC->K;
         Math::Matrix3x3d intrinsics = CvMatfToMatrix3x3d(matIntrinsics);
         double residual = 0.0;
         Math::Matrix< double, 3, 4 > poseMat(mathPose);
         Math::Matrix3x4d projectionMatrix = boost::numeric::ublas::prod(intrinsics, poseMat);
         std::vector<Math::Vector3d> points3d;
         for (size_t i = 0; i < f.N; i++)
         {
            MapPoint * pMP = f.mvpMapPoints[i];
            cv::Mat wPos = pMP->GetWorldPos();
            if (pMP)
            {
               cv::KeyPoint & kp = f.mvKeysUn[i];
               Math::Vector3d point3d(wPos.at<float>(0), wPos.at<float>(1), wPos.at<float>(2));
               points3d.push_back(point3d);
               Math::Vector4d hom(point3d[0], point3d[1], point3d[2], 1.0);
               Math::Vector3d tmp = boost::numeric::ublas::prod(projectionMatrix, hom);
               if (tmp[2] == 0.0) LOG4CPP_INFO(logger, "Divide by Zero!!!");
               tmp = tmp / tmp[2];
               residual += (tmp[0] - kp.pt.x) * (tmp[0] - kp.pt.x) + (tmp[1] - kp.pt.y) * (tmp[1] - kp.pt.y);
            }
         }

         /* If m_sensor == SensorType::STEREO, the images are pre-rectified and the right image is
            used to calculate the depth of a KeyPoint. Thus, both images will observe the same 
            world points. It is not clear whether using Algorithm::PoseEstimation2D3D::multipleCameraPoseError
            is a significant advantage in error pose calculation.
         */

         Math::Matrix<double, 6, 6> covar = Algorithm::PoseEstimation2D3D::singleCameraPoseError(mathPose, points3d, intrinsics, residual);
         covar(0, 0) += m_addErrorX * m_addErrorX;
         covar(1, 1) += m_addErrorY * m_addErrorY;
         covar(2, 2) += m_addErrorZ * m_addErrorZ;
         m_pushErrorPose.send(Measurement::ErrorPose(t, Math::ErrorPose(mathPose, covar)));
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


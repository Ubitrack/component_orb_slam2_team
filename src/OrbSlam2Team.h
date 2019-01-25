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

#ifndef __Ubitrack_OrbSlam2Team_INCLUDED__
#define __Ubitrack_OrbSlam2Team_INCLUDED__

#include <string>
#include <strstream>

//#include <boost/thread.hpp>
//#include <boost/bind.hpp>
//#include <boost/scoped_ptr.hpp>
//#include <boost/scoped_array.hpp>
//#include <boost/numeric/ublas/blas.hpp>

#include <utDataflow/Module.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
//#include <utUtil/OS.h>
//#include <utUtil/TracingProvider.h>
#include <utUtil/BlockTimer.h>
#include <opencv/cv.h>
#include <utVision/Image.h>
//#include <utVision/Undistortion.h>
#include <utDataflow/TriggerComponent.h>
#include <utDataflow/TriggerInPort.h>
#include <utDataflow/TriggerOutPort.h>
//#include <utAlgorithm/Homography.h>
//#include <utAlgorithm/PoseEstimation2D3D/PlanarPoseEstimation.h>
//#include <utAlgorithm/Projection.h>
//#include <boost/numeric/ublas/blas.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

// ORB_SLAM2_TEAM includes
#include <ORBVocabulary.h>
#include <MapperServer.h>
#include <Tracking.h>
#include <FrameDrawer.h>

namespace Ubitrack {
   namespace Components {

      using namespace Ubitrack;
      using namespace ORB_SLAM2_TEAM;


      // Module Key
      MAKE_NODEATTRIBUTEKEY_DEFAULT(MapperKey, int, "Mapper", "mapperId", 1);


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


      class OrbSlam2TeamComponent;


      class OrbSlam2TeamModule
         : public Dataflow::Module< MapperKey, TrackerKey, OrbSlam2TeamModule, OrbSlam2TeamComponent >
      {
      public:

         /** UTQL constructor */
         OrbSlam2TeamModule(const MapperKey& key, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, FactoryHelper* pFactory);

         virtual void startModule();

         virtual void stopModule();

         // override factory method to create components
         boost::shared_ptr<OrbSlam2TeamComponent> createComponent(const string& type, const string& name,
            boost::shared_ptr< Graph::UTQLSubgraph > subgraph, const TrackerKey& key, OrbSlam2TeamModule* pModule);

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


      class OrbSlam2TeamBase : public Dataflow::TriggerComponent
      {
      public:

         OrbSlam2TeamBase(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph, SensorType sensor);

         virtual void start();

         virtual void stop();

         virtual void compute(Measurement::Timestamp t) = 0;

         Measurement::PositionList pullMapPoints( Ubitrack::Measurement::Timestamp t );

         Measurement::PoseList pullKeyFrames( Ubitrack::Measurement::Timestamp t );

      protected:

         Dataflow::PushSupplier< Measurement::ImageMeasurement > m_pushImgDebug;
         Dataflow::TriggerOutPort< Measurement::Pose > m_outPose;
         Dataflow::PushSupplier< Measurement::ErrorPose > m_pushErrorPose;
         Dataflow::PullSupplier< Measurement::PositionList > m_pullMapPoints;
         Dataflow::PullSupplier< Measurement::PoseList > m_pullKeyFrames;

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

      private:

         SensorType m_sensor;
         static unsigned int m_maxTrackers;

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
      * \c Ubitrack::Measurement::Pose Baseline
      */
      class OrbSlam2TeamStereo : public OrbSlam2TeamBase
      {
      public:

         OrbSlam2TeamStereo(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph);

         virtual void start();

         virtual void compute(Measurement::Timestamp t);

      protected:

         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageL;
         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageR;
         Dataflow::PushSupplier< Measurement::Pose > m_outBaseline;

      };


      /**
      * @ingroup vision_components
      *
      * @par Input Ports
      * \c Ubitrack::Measurement::ImageMeasurement ImageInput
      *
      * @par Output Ports
      * \c Ubitrack::Measurement::Pose Output
      * \c Ubitrack::Measurement::ErrorPose OutputError
      */
      class OrbSlam2TeamMono : public OrbSlam2TeamBase
      {
      public:

         OrbSlam2TeamMono(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph);

         virtual void compute(Measurement::Timestamp t);

      protected:

         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImage;

      };


      /**
      * @ingroup vision_components
      *
      * @par Input Ports
      * \c Ubitrack::Measurement::ImageMeasurement ImageInput
      * \c Ubitrack::Measurement::ImageMeasurement ImageInputD
      *
      * @par Output Ports
      * \c Ubitrack::Measurement::Pose Output
      * \c Ubitrack::Measurement::ErrorPose OutputError
      * \c Ubitrack::Measurement::Pose Baseline
      */
      class OrbSlam2TeamRgbd : public OrbSlam2TeamBase
      {
      public:

         OrbSlam2TeamRgbd(const string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph);

         virtual void start();

         virtual void compute(Measurement::Timestamp t);

      protected:

         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageRgb;
         Dataflow::TriggerInPort< Measurement::ImageMeasurement > m_inImageD;
         Dataflow::PushSupplier< Measurement::Pose > m_outBaseline;

      };

   } // namespace Components
} // namespace Ubitrack


UBITRACK_REGISTER_COMPONENT(Ubitrack::Dataflow::ComponentFactory * const cf) {
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamStereo >("OrbSlam2TeamStereo");
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamMono >("OrbSlam2TeamMono");
   cf->registerComponent< Ubitrack::Components::OrbSlam2TeamRgbd >("OrbSlam2TeamRgbd");

   // can't use the module-component technique until Module supports TriggerComponent
   //cf->registerModule< Ubitrack::Components::OrbSlam2TeamModule >("OrbSlam2TeamStereo");
   //cf->registerModule< Ubitrack::Components::OrbSlam2TeamModule >("OrbSlam2TeamMono");
   //cf->registerModule< Ubitrack::Components::OrbSlam2TeamModule >("OrbSlam2TeamRgbd");
}


#endif
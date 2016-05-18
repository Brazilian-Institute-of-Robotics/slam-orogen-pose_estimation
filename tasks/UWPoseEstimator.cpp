/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "UWPoseEstimator.hpp"

using namespace pose_estimation;

UWPoseEstimator::UWPoseEstimator(std::string const& name)
    : UWPoseEstimatorBase(name)
{
}

UWPoseEstimator::UWPoseEstimator(std::string const& name, RTT::ExecutionEngine* engine)
    : UWPoseEstimatorBase(name, engine)
{
}

UWPoseEstimator::~UWPoseEstimator()
{
}

void UWPoseEstimator::acceleration_samplesTransformerCallback(const base::Time& ts, const base::samples::RigidBodyAcceleration& acceleration_samples_sample)
{
    handleMeasurement(ts, acceleration_samples_sample);
}

void UWPoseEstimator::depth_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &depth_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, depth_samples_sample, measurement_mask, _pressure_sensor2body);
}

void UWPoseEstimator::orientation_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberRoll] = 1;
    measurement_mask[BodyStateMemberPitch] = 1;
    measurement_mask[BodyStateMemberYaw] = 1;
    handleMeasurement(ts, orientation_samples_sample, measurement_mask, _imu2body);

    angular_velocity = orientation_samples_sample.angular_velocity;
}

void UWPoseEstimator::lbl_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &lbl_position_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberX] = 1;
    measurement_mask[BodyStateMemberY] = 1;
    measurement_mask[BodyStateMemberZ] = 1;
    handleMeasurement(ts, lbl_position_samples_sample, measurement_mask, _lbl2body);
}

void UWPoseEstimator::xy_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &xy_position_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberX] = 1;
    measurement_mask[BodyStateMemberY] = 1;
    handleMeasurement(ts, xy_position_samples_sample, measurement_mask);
}

void UWPoseEstimator::dvl_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &dvl_velocity_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberVx] = 1;
    measurement_mask[BodyStateMemberVy] = 1;
    measurement_mask[BodyStateMemberVz] = 1;
    handleMeasurement(ts, dvl_velocity_samples_sample, measurement_mask, _dvl2body);
}

void UWPoseEstimator::model_velocity_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &model_velocity_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberVx] = 1;
    measurement_mask[BodyStateMemberVy] = 1;
    handleMeasurement(ts, model_velocity_samples_sample, measurement_mask);
}


void UWPoseEstimator::gps_position_samplesTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_position_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberX] = 1;
    measurement_mask[BodyStateMemberY] = 1;

    handleMeasurement(ts, gps_position_samples_sample, measurement_mask,  _gps2body);
}


void UWPoseEstimator::xyz_position_samplesTransformerCallback( const base::Time &ts, const ::base::samples::RigidBodyState &xyz_position_samples_sample)
{
    MemberMask measurement_mask = MemberMask::Zero();
    measurement_mask[BodyStateMemberX] = 1;
    measurement_mask[BodyStateMemberY] = 1;
    measurement_mask[BodyStateMemberZ] = 1;

    handleMeasurement(ts, xyz_position_samples_sample, measurement_mask);
}
  
/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See UWPoseEstimator.hpp for more detailed
// documentation about them.

bool UWPoseEstimator::configureHook()
{
    if (! UWPoseEstimatorBase::configureHook())
        return false;
    
    source_frame = _body_frame.get();
    angular_velocity = Eigen::Vector3d::Zero();
    
    return true;
}
bool UWPoseEstimator::startHook()
{
    if (! UWPoseEstimatorBase::startHook())
        return false;
    return true;
}
void UWPoseEstimator::updateHook()
{
    UWPoseEstimatorBase::updateHook();
    
    // verify stream aligner status
    verifyStreamAlignerStatus(_transformer);
    
    // update and write new state
    // integrate measurements
    try
    {
	pose_estimator->integrateMeasurements();
    }
    catch (std::runtime_error e)
    {
	RTT::log(RTT::Error) << "Failed to integrate measurements: " << e.what() << RTT::endlog();
    }
    
    // write estimated body state
    StateAndCovariance current_state;
    if(pose_estimator->getEstimatedState(current_state))
    {
        base::samples::RigidBodyState body_state;
        BodyStateMeasurement::toRigidBodyState(current_state.mu, current_state.cov, body_state);
        body_state.angular_velocity = angular_velocity;
        current_body_state = body_state;
	body_state.time = pose_estimator->getLastMeasurementTime();
	body_state.targetFrame = _target_frame.get();
	body_state.sourceFrame = source_frame;
	_pose_samples.write(body_state);
    }
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        RBSFilter::state(new_state);
    }
}
void UWPoseEstimator::errorHook()
{
    UWPoseEstimatorBase::errorHook();
}
void UWPoseEstimator::stopHook()
{
    UWPoseEstimatorBase::stopHook();
}
void UWPoseEstimator::cleanupHook()
{
    UWPoseEstimatorBase::cleanupHook();
}

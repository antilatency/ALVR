#include "antilatency.h"

#include <unistd.h>
#include <sstream>
#include <memory>
#include <chrono>
#include <Antilatency.InterfaceContract.LibraryLoader.h>

#include "../../../../../alvr/client_core/cpp/glm/gtx/quaternion.hpp"
#include "../../../../../alvr/client_core/cpp/glm/gtx/rotate_vector.hpp"
#include "VrApi_Types.h"

using namespace Antilatency;

AntilatencyManager::AntilatencyManager(JNIEnv* env, jobject activity) {
    if (loadLibraries()) {
        if (env != nullptr) {
            JavaVM *jvm;
            env->GetJavaVM(&jvm);
            LOGI("Antilatency: JavaVM was taken");

            auto adnJni = _adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
            LOGI("Antilatency: adn wrapped interface was taken");

            if (jvm == nullptr)
                LOGI("Antilatency: jvm is nullptr");
            if (activity == nullptr)
                LOGI("Antilatency: activity is nullptr");
            if (adnJni == nullptr)
                LOGI("Antilatency: adnJni is nullptr");
            try {
                adnJni.initJni(jvm, activity);
            }
            catch (Antilatency::InterfaceContract::Exception &ex) {
                LOGI("Antilatency: Antilatency::InterfaceContract::Exception");
                LOGI("Antilatency: %s", ex.what());
            }
            catch (...) {
                LOGI("Antilatency: Any catcher");
            }

            LOGI("Antilatency: adn jni was initialized");

            auto antilatencyStorageClientJni = _antilatencyStorageLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
            LOGI("Antilatency: storageclient wrapped interface was taken");
            antilatencyStorageClientJni.initJni(jvm, activity);

            LOGI("Antilatency: Device Network ver.: %s", _adnLibrary.getVersion().data());
            auto filter = _adnLibrary.createFilter();
            LOGI("Antilatency: Filter was created");

            filter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
            filter.addIpDevice(Antilatency::DeviceNetwork::Constants::AllIpDevicesIp,
                               Antilatency::DeviceNetwork::Constants::AllIpDevicesMask);

            _deviceNetwork = _adnLibrary.createNetwork(filter);
            LOGI("Antilatency: network was created");

            auto environmentCode = _antilatencyStorageLibrary.getLocalStorage().read("environment",
                                                                                     "default");
            LOGI("Antilatency: environmentCode was read");

            _environment = _environmentSelectorLibrary.createEnvironment(environmentCode);
            LOGI("Antilatency: environment was created");

            //Get placement code from AltSystem app (default in this case) and create placement from it.
            auto placementCode = _antilatencyStorageLibrary.getLocalStorage().read("placement",
                                                                                   "default");
            LOGI("Antilatency: placementCode was read");

            _placement = _altTrackingLibrary.createPlacement(placementCode);
            LOGI("Antilatency: placement was read");

            _trackingAlignment = _trackingAlignmentLibrary.createTrackingAlignment(
                    MathUtils::FloatFromQ(_placement.rotation), _extrapolationTime);
            LOGI("Antilatency: alignment was created");

            _rigPose = Antilatency::Math::floatP3Q();
            _startTime = std::chrono::high_resolution_clock::now();
        }
        else {
            throw Antilatency::InterfaceContract::Exception("Variable env is nullptr");
        }
    }
    else {
        throw Antilatency::InterfaceContract::Exception("Libraries were not loaded");
    }
}

bool AntilatencyManager::loadLibraries() {
    try {
        _adnLibrary = InterfaceContract::getLibraryInterface<DeviceNetwork::ILibrary>(
                "libAntilatencyDeviceNetwork.so");
        LOGI("Antilatency: adn library was created");

        _antilatencyStorageLibrary = InterfaceContract::getLibraryInterface<StorageClient::ILibrary>(
                "libAntilatencyStorageClient.so");
        LOGI("Antilatency: Storage library was created");

        _environmentSelectorLibrary = InterfaceContract::getLibraryInterface<Alt::Environment::Selector::ILibrary>(
                "libAntilatencyAltEnvironmentSelector.so");
        LOGI("Antilatency: Environment Selector library was created");

        _altTrackingLibrary = InterfaceContract::getLibraryInterface<Alt::Tracking::ILibrary>(
                "libAntilatencyAltTracking.so");
        LOGI("Antilatency: AltTracking library was created");

        _trackingAlignmentLibrary = InterfaceContract::getLibraryInterface<TrackingAlignment::ILibrary>(
                "libAntilatencyTrackingAlignment.so");
        LOGI("Antilatency: AltTrackingAlignment library was created");
    } catch (std::exception& exception) {
        LOGE("Antilatency: Exception in loadLibraries: %s", exception.what());
        return false;
    }

    return true;
}

void AntilatencyManager:: updateNodes() {
    //Get current Antilatency Device Network update id. It will be incremented every time any supported device is added or removed.
    auto updateId = _deviceNetwork.getUpdateId();

    if (_updateId != updateId) {
        _updateId = updateId;
        LOGI("Antilatency Device Network update id has been incremented: %u", _updateId);

        //Create tracking cotask constructor to check if node supports tracking and start tracking task on node.
        auto cotaskConstructor = _altTrackingLibrary.createTrackingCotaskConstructor();

        //Get all currently connected nodes that supports tracking task.
        auto nodes = cotaskConstructor.findSupportedNodes(_deviceNetwork);
        for (auto node : nodes) {
            //Check if node is idle, we cannot start task on invalid nodes or on nodes that already has task started on it.
            if (_deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {
                handleNode(node);
            }
        }
    }

    auto rmIter = std::remove_if(_trackers.begin(), _trackers.end(), [this](AntilatencyTracker& tracker) {
        if (tracker.cotask.isTaskFinished()) {
            LOGI("Tracking node offline: %s", tracker.serialNumber.data());
            return true;
        } else {
            updateTracker(tracker);
            return false;
        }
    });

    _trackers.erase(rmIter, _trackers.end());
}

void AntilatencyManager::handleNode(Antilatency::DeviceNetwork::NodeHandle node) {
    uint8_t trackerType = AntilatencyTracker::TYPE_UNKNOWN;
    try {
        std::string serialNumber = _deviceNetwork.nodeGetStringProperty(node, DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
        std::string tag = _deviceNetwork.nodeGetStringProperty(_deviceNetwork.nodeGetParent(node), "Tag");
        if (tag == "HMD") {
            trackerType = AntilatencyTracker::TYPE_HMD;
        } else if (tag == "LeftHand") {
            trackerType = AntilatencyTracker::TYPE_LEFT_CONTROLLER;
        } else if (tag == "RightHand") {
            trackerType = AntilatencyTracker::TYPE_RIGHT_CONTROLLER;
        } else {
            LOGE("Unknown tag: %s", tag.data());
            return;
        }

        auto cotaskConstructor = _altTrackingLibrary.createTrackingCotaskConstructor();
        auto trackingCotask = cotaskConstructor.startTask(_deviceNetwork, node, _environment);
        LOGI("Antilatency: cotask was started");
        _trackers.push_back(AntilatencyTracker{trackerType, serialNumber, trackingCotask });
        LOGI("Tracking node online [%s]: %s", tag.data(), serialNumber.data());

    } catch (InterfaceContract::Exception &e) {
        LOGE("Handle node failed: %s", e.message().data());
    }
}

void AntilatencyManager::updateTracker(AntilatencyTracker& tracker) {
    if (tracker.cotask.isTaskFinished()) {
        return;
    }
    switch (tracker.type) {
        case AntilatencyTracker::TYPE_HMD:
            _antilatencyTrackingData.head = proceedTrackingAlignment(tracker);
            break;
        case AntilatencyTracker::TYPE_LEFT_CONTROLLER:
            _antilatencyTrackingData.leftHand = tracker.cotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
            break;
        case AntilatencyTracker::TYPE_RIGHT_CONTROLLER:
            _antilatencyTrackingData.rightHand = tracker.cotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
            break;
        default:
            break;
    }
}

Antilatency::Alt::Tracking::State AntilatencyManager::proceedTrackingAlignment(AntilatencyTracker& tracker) {
    if (!tracker.cotask) {
        return Antilatency::Alt::Tracking::State();
    }

    auto trackingState = tracker.cotask.getState(Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);

    auto resultState = Antilatency::Alt::Tracking::State();

    if (!_trackingAlignment) {
        return Antilatency::Alt::Tracking::State();
    }

    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization) {
        return Antilatency::Alt::Tracking::State();
    }

    if (trackingState.stability.stage == Alt::Tracking::Stage::Tracking6Dof && trackingState.stability.value > 0.075f) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeSinceStart = currentTime - _startTime;

        auto dTrackingState = MathUtils::FloatFromQ(trackingState.pose.rotation);
        auto dMRigRotation = MathUtils::FloatFromQ(_rigPose.rotation);

        auto result = _trackingAlignment.update(
                dTrackingState,
                dMRigRotation,
                timeSinceStart.count()
        );

        _externalSpace = result;

        _extrapolationTime = static_cast<float>(result.timeBAheadOfA);
        _placement.rotation = MathUtils::FloatFromQ(result.rotationARelativeToB);

        _trackingSpaceRotation = MathUtils::FloatFromQ(result.rotationBSpace);
    }

    trackingState = tracker.cotask.getExtrapolatedState(_placement, _extrapolationTime);

    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization) {
        return Antilatency::Alt::Tracking::State();
    }

    LOGI("Antilatency:: _trackingSpaceRotation = (%f,%f,%f, %f)", _trackingSpaceRotation.x, _trackingSpaceRotation.y, _trackingSpaceRotation.z, _trackingSpaceRotation.w);
    auto resultRotation = MathUtils::MultiplyQuat(_trackingSpaceRotation, _rigPose.rotation);

    resultState.pose.rotation = resultRotation;

    resultState.pose.position = trackingState.pose.position;

    _lastHMDPosition =  resultState.pose.position;

    return resultState;
}

Antilatency::Math::floatP3Q AntilatencyManager::getPlacement() {
    return _placement;
}

std::optional<Antilatency::TrackingAlignment::State> AntilatencyManager::getExternalSpace() {
    return _externalSpace;
}

void AntilatencyManager::controllerRotationCorrection(Antilatency::Math::floatQ BControllerOrientation, Antilatency::Math::floatQ& correctionRotationResult) {
    BControllerOrientation.z *= -1.0f;
    BControllerOrientation.w *= -1.0f;

    if (_externalSpace.has_value()) {
        correctionRotationResult = MathUtils::MultiplyQuat(_externalSpace.value().rotationBSpace,
                                                           BControllerOrientation);
    } else {
        correctionRotationResult = BControllerOrientation;
    }
    correctionRotationResult.z *= -1.0f;
    correctionRotationResult.w *= -1.0f;
}

const Antilatency::Math::float3 AntilatencyManager::controllerVelocityCorrection(const Antilatency::Math::float3& vectorForCorrection) {
    Antilatency::Math::float3 result = {0,0,0};

    if (_externalSpace.has_value() && (MathUtils::ToGLMVec3(vectorForCorrection) != MathUtils::ToGLMVec3(Antilatency::Math::float3{0, 0, 0}))) {
        auto glmVectorBSpacePosition = MathUtils::ToGLMVec3(vectorForCorrection);
        glmVectorBSpacePosition.z *= -1;

        auto glmRotationBSpace = MathUtils::ToGLMQuat(_externalSpace.value().rotationBSpace);

        result = MathUtils::Float3FromPosition(glm::rotate(glmRotationBSpace, glmVectorBSpacePosition));

        result.z *= -1.0f;
    }
    else {
        LOGE("Antilatency: Error, _externalSpace hasn't value");
    }

    return result;
}

const Antilatency::Math::float3 AntilatencyManager::controllerPositionCorrection(const Antilatency::Math::float3& vectorForCorrection, int controllerID) {
    Antilatency::Math::float3 result = {0,0,0};

    if (_externalSpace.has_value() && (MathUtils::ToGLMVec3(vectorForCorrection) != MathUtils::ToGLMVec3(Antilatency::Math::float3{0, 0, 0}))) {
        auto glmVectorBSpacePosition = MathUtils::ToGLMVec3(vectorForCorrection);
        glmVectorBSpacePosition.z *= -1;

        auto glmRotationBSpace = MathUtils::ToGLMQuat(_externalSpace.value().rotationBSpace);
        auto lastHMDBPosition = _lastHMDOwnPosition;

        lastHMDBPosition.z *= -1;
        _positionOffset = MathUtils::Float3FromPosition(glm::rotate(glmRotationBSpace, glmVectorBSpacePosition - MathUtils::ToGLMVec3(lastHMDBPosition)));
        result = MathUtils::Float3FromPosition(MathUtils::ToGLMVec3(_lastHMDPosition) + MathUtils::ToGLMVec3(_positionOffset));

        result.z *= -1.0f;
    }
    else {
        LOGE("Antilatency: Error, _externalSpace hasn't value");
    }

    return result;
}

//Antilatency::Alt::Tracking::State AntilatencyManager::proceedTrackingAlignment(AntilatencyTracker &tracker) {
//    auto currentTime = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double> timeSinceStart = currentTime - _startTime;
//
//    //auto bPositionReceived =
//
//    bool altTrackingActive = tracker.cotask ? true : false;
//
//    if (!altTrackingActive)
//        return Antilatency::Alt::Tracking::State();
//
//    auto trackingState = tracker.cotask.getState(Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
//
//    auto resultState = Antilatency::Alt::Tracking::State();
//
//    //TODO _lerpRotation
//
//    if (_trackingAlignment != nullptr && trackingState.stability.stage == Alt::Tracking::Stage::Tracking6Dof && trackingState.stability.value >= 0.075f) {
//        auto dTrackingState = MathUtils::FloatFromQ(trackingState.pose.rotation);
//        auto dMRigRotation = MathUtils::FloatFromQ(_rigPose.rotation);
//
//        auto result = _trackingAlignment.update(
//                dTrackingState,
//                dMRigRotation,
//                timeSinceStart.count()
//        );
//
//        _externalSpace = result; // ???
//
//        _extrapolationTime      = static_cast<float>    (result.timeBAheadOfA);
//        _placement.rotation     = MathUtils::FloatFromQ (result.rotationARelativeToB);
//        _trackingSpaceRotation  = MathUtils::FloatFromQ (result.rotationBSpace);
//    }
//
////    altTrackingActive = tracker.cotask ? true : false;
////
////    if (!altTrackingActive)
////        return Antilatency::Alt::Tracking::State();
//
//    trackingState = tracker.cotask.getExtrapolatedState(_placement, _extrapolationTime);
//
//    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization) {
//        return Antilatency::Alt::Tracking::State();
//    }
//
////  suppose _lerpRotation & _lerpPosition is true
////  => always is false
////  if (!_lerpRotation) {
////      _bSpace.localRotation = trackingState.pose.rotation;
////      _b.localRotation = Quaternion.identity;
////  }
//
//    if (trackingState.stability.stage == Antilatency::Alt::Tracking::Stage::Tracking6Dof && /*bPositionReceived && */trackingState.stability.value >= 0.075f) {
//        auto a = trackingState.pose.position;
////      auto b = ;
//
//        Antilatency::Math::float3 averagePositionInASpace;
//
//        if (!_altInitialPositionApplied) {
//            averagePositionInASpace = a;
//            _altInitialPositionApplied = true;
//        } else {
//            averagePositionInASpace.x = (b.x * 0.6f + a.x * trackingState.stability.value) / (trackingState.stability.value + 0.6f);
//            averagePositionInASpace.y = (b.y * 0.6f + a.y * trackingState.stability.value) / (trackingState.stability.value + 0.6f);
//            averagePositionInASpace.z = (b.z * 0.6f + a.z * trackingState.stability.value) / (trackingState.stability.value + 0.6f);
//        }
//
//        _bSpace.position += averagePositionInASpace - b;
//
//        resultState.pose.position = _bSpace.position;
//    }
//
//    return resultState; // or inside if
//}


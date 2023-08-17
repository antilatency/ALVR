#pragma once

#ifndef CLOUDVRCLIENT_ANTILATENCY_H
#define CLOUDVRCLIENT_ANTILATENCY_H

#include <string>
#include <jni.h>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <optional>
#include <Antilatency.Api.h>

#include "MathUtils.h"
#include "packet_types.h"
#include "../../../../../alvr/client_core/cpp/utils.h"
#include "../../../../../alvr/client_core/cpp/glm/gtc/type_ptr.hpp"

struct AntilatencyTracker {
    enum {
        TYPE_UNKNOWN,
        TYPE_HMD,
        TYPE_LEFT_CONTROLLER,
        TYPE_RIGHT_CONTROLLER
    };
    uint8_t type;
    std::string serialNumber;
    Antilatency::Alt::Tracking::ITrackingCotask cotask;
};

//struct AntilatencyBracer {
//    enum {
//        TYPE_UNKNOWN,
//        TYPE_LEFT_CONTROLLER,
//        TYPE_RIGHT_CONTROLLER
//    };
//    uint8_t type;
//    std::string serialNumber;
//    Antilatency::Bracer::ICotask cotask;
//};

struct AntilatencyTrackingData {
    Antilatency::Alt::Tracking::State head;
    Antilatency::Alt::Tracking::State leftHand;
    Antilatency::Alt::Tracking::State rightHand;
    ////////////////////////////////////////////
//    bool rightClick;
//    bool leftClick;
//    float rightTouch;
//    float leftTouch;
};

class AntilatencyManager {
public:
    AntilatencyManager(JNIEnv* env, jobject activity);

    inline const AntilatencyTrackingData& getTrackingData();

    inline bool okTracker(const std::function<bool(AntilatencyTracker&)>& predicate);

    template<typename T, typename A>
    void setRigPose(T rigPosition, A rigRotation, double extrapolationTime);

    Antilatency::Math::floatP3Q getPlacement();
    std::optional<Antilatency::TrackingAlignment::State> getExternalSpace();

    void correctPositionAndRotation(Antilatency::Alt::Tracking::State& state);
    void controllerRotationCorrection(Antilatency::Math::floatQ BControllerOrientation, Antilatency::Math::floatQ& correctionRotationResult);
    const Antilatency::Math::float3 controllerPositionCorrection(const Antilatency::Math::float3& vectorForCorrection);
    const Antilatency::Math::float3 controllerVelocityCorrection(const Antilatency::Math::float3& vectorForCorrection);
private:
    bool loadLibraries();

    void updateNodes();

    void handleNode(Antilatency::DeviceNetwork::NodeHandle node);
    //void handleBracerNode(Antilatency::DeviceNetwork::NodeHandle node);

    void updateTracker(AntilatencyTracker& tracker);
    //void updateBracer(AntilatencyBracer& bracer);

    Antilatency::Alt::Tracking::State proceedTrackingAlignment(AntilatencyTracker& tracker);
private:
    Antilatency::DeviceNetwork::ILibrary _adnLibrary;
    Antilatency::StorageClient::ILibrary _antilatencyStorageLibrary;
    Antilatency::Alt::Environment::Selector::ILibrary _environmentSelectorLibrary;
    Antilatency::Alt::Tracking::ILibrary _altTrackingLibrary;
    Antilatency::TrackingAlignment::ILibrary _trackingAlignmentLibrary;
    //Antilatency::Bracer::ILibrary _bracerLibrary;

    Antilatency::Math::floatP3Q _rigPose{};

    uint32_t _updateId = -1;

    std::chrono::high_resolution_clock::time_point _startTime;

    Antilatency::DeviceNetwork::INetwork _deviceNetwork;
    Antilatency::Alt::Environment::IEnvironment _environment;
    Antilatency::Math::floatP3Q _placement{};

    Antilatency::TrackingAlignment::ITrackingAlignment _trackingAlignment;

    float _extrapolationTime = 0.045f;
    float _headsetTime = 0.0f;

    Antilatency::Math::floatQ _trackingSpaceRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    AntilatencyTrackingData _antilatencyTrackingData{};

    std::vector<AntilatencyTracker> _trackers;
    //std::vector<AntilatencyBracer> _bracers;
    std::optional<Antilatency::TrackingAlignment::State> _externalSpace;

    Antilatency::Math::float3 _lastHMDOwnPosition{};
    Antilatency::Math::float3 _lastHMDPosition{};

    Antilatency::Math::float3 _positionOffset{};
};

const AntilatencyTrackingData& AntilatencyManager::getTrackingData() {
    updateNodes();
    return _antilatencyTrackingData;
}

bool AntilatencyManager::okTracker(const std::function<bool(AntilatencyTracker&)>& predicate) {
    auto foundTracker = std::find_if(_trackers.begin(),
                                     _trackers.end(),
                                     predicate
    );

    if (foundTracker == _trackers.end()) {
        return false;
    } else {
        if (foundTracker->cotask == nullptr || foundTracker->cotask.isTaskFinished()) {
            return false;
        } else {
            auto state = foundTracker->cotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);

            return state.stability.stage != Antilatency::Alt::Tracking::Stage::InertialDataInitialization;
        }
    }
}

template<typename T, typename A>
void AntilatencyManager::setRigPose(T rigPosition, A rigRotation, double extrapolationTime) {
    _lastHMDOwnPosition = MathUtils::Float3FromPosition(rigPosition);

    _rigPose.position = MathUtils::Float3FromPosition(rigPosition);
    _rigPose.position.z *= -1;

    _rigPose.rotation = MathUtils::FloatFromQ(rigRotation);
    _rigPose.rotation.w *= -1;
    _rigPose.rotation.z *= -1;

    _headsetTime = static_cast<float>(extrapolationTime);
}

#endif // CLOUDVRCLIENT_ANTILATENCY_H
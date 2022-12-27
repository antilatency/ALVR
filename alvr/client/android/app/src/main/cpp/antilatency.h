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

#include <glm/gtc/type_ptr.hpp>

#include <Antilatency.Api.h>
#include "MathUtils.h"


#include "packet_types.h"
#include "utils.h"



struct MathUtils;

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

struct AntilatencyTrackingData {
    Antilatency::Alt::Tracking::State head;
    Antilatency::Alt::Tracking::State leftHand;
    Antilatency::Alt::Tracking::State rightHand;
};

class AntilatencyManager {
public:
    AntilatencyManager(JNIEnv* env, jobject activity);

    ~AntilatencyManager();

    inline const AntilatencyTrackingData &getTrackingData();

    inline bool hasAntilatency();

    template<typename T, typename A>
    void setRigPose(T rigPosition, A rigRotation, double extrapolationTime);
    Antilatency::Math::floatP3Q getPlacement();
    std::optional<Antilatency::TrackingAlignment::State> getExternalSpace();

    void controllerRotationCorrection(Antilatency::Math::floatQ BControllerOrientation, Antilatency::Math::floatQ& correctionRotationResult);
    const Antilatency::Math::float3 controllerPositionCorrection(const Antilatency::Math::float3& vectorForCorrection, int controllerID);
    const Antilatency::Math::float3 controllerVelocityCorrection(const Antilatency::Math::float3& vectorForCorrection);

    Antilatency::Math::float3 getLastControllerPosition(int controllerID) const;
private:
    void initJni(Antilatency::InterfaceContract::IInterface obj, JavaVM* jvm, jobject instance);
    void doTracking();

    void handleNode(Antilatency::DeviceNetwork::NodeHandle node);

    void updateTracker(AntilatencyTracker& tracker);

    void startTrackingAlignment();
    void stopTrackingAlignment();
    Antilatency::Alt::Tracking::State proceedTrackingAlignment(AntilatencyTracker &tracker);

private:
    Antilatency::DeviceNetwork::ILibrary m_adnLibrary;
    Antilatency::StorageClient::ILibrary m_antilatencyStorageLibrary;
    Antilatency::Alt::Environment::Selector::ILibrary m_environmentSelectorLibrary;
    Antilatency::Alt::Tracking::ILibrary m_altTrackingLibrary;
    Antilatency::TrackingAlignment::ILibrary m_trackingAlignmentLibrary;

    Antilatency::Math::floatP3Q m_rigPose;

    uint32_t updateId = -1;

    float m_bQuality = 0.4f;
    bool m_altInitialPositionApplied = false;

    std::chrono::high_resolution_clock::time_point m_startTime;

    std::thread m_trackingThread;
    bool m_trackingThreadRunning = false;

    Antilatency::DeviceNetwork::INetwork m_deviceNetwork;
    Antilatency::Alt::Environment::IEnvironment m_environment;
    Antilatency::Math::floatP3Q m_placement;

    Antilatency::TrackingAlignment::ITrackingAlignment m_trackingAlignment;

    float m_extrapolationTime = 0.045f;
    float m_headsetTime = 0.0f;

    Antilatency::Math::float3 trackingSpacePosition;
    Antilatency::Math::floatQ trackingSpaceRotation = {0.0f, 0.0f, 0.0f, 1.0f};

    AntilatencyTrackingData m_antilatencyTrackingData;
    std::mutex m_poseMutex;

    std::vector<AntilatencyTracker> m_trackers;
    std::optional<Antilatency::TrackingAlignment::State> externalSpace;

    Antilatency::Math::float3 lastHMDOwnPosition;


    Antilatency::Math::float3 lastHMDPosition;

    Antilatency::Math::float3 positionOffset;
    Antilatency::Math::float3 lastControllerPositionASpace[2];

    size_t iterationID = 0;
};

const AntilatencyTrackingData &AntilatencyManager::getTrackingData() {
    doTracking();
    return m_antilatencyTrackingData;
}

bool AntilatencyManager::hasAntilatency() {
    return m_trackers.size() > 0;
}

template<typename T, typename A>
void AntilatencyManager::setRigPose(T rigPosition, A rigRotation, double extrapolationTime){
    lastHMDOwnPosition = MathUtils::Float3FromPosition(rigPosition);

    m_rigPose.position = MathUtils::Float3FromPosition(rigPosition);
    m_rigPose.position.z *= -1;

    m_rigPose.rotation = MathUtils::FloatFromQ(rigRotation);
    m_rigPose.rotation.w *= -1;
    m_rigPose.rotation.z *= -1;

    m_headsetTime = static_cast<float>(extrapolationTime);
};

#endif // CLOUDVRCLIENT_ANTILATENCY_H
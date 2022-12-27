#include "antilatency.h"

#include <unistd.h>
#include <sstream>
#include <memory>
#include <chrono>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <Antilatency.InterfaceContract.LibraryLoader.h>

#define LOGALT(...) __android_log_print(ANDROID_LOG_DEBUG, "Cloud VR Antilatency", __VA_ARGS__)
#define LOGALTI(...) __android_log_print(ANDROID_LOG_INFO, "Cloud VR Antilatency", __VA_ARGS__)
#define LOGALTE(...) __android_log_print(ANDROID_LOG_ERROR, "Cloud VR Antilatency", __VA_ARGS__)


using namespace Antilatency;

void AntilatencyManager::initJni(Antilatency::InterfaceContract::IInterface obj, JavaVM* jvm, jobject instance) {
    auto jni = obj.queryInterface<AndroidJniWrapper::IAndroidJni>();
    if (jni == nullptr){
        return;
    }

    jni.initJni(jvm, instance);
}

AntilatencyManager::AntilatencyManager(JNIEnv* env, jobject activity) {
    m_adnLibrary = InterfaceContract::getLibraryInterface<DeviceNetwork::ILibrary>("libAntilatencyDeviceNetwork.so");
    LOGI("Antilatency: adn library was created");

    m_antilatencyStorageLibrary = InterfaceContract::getLibraryInterface<StorageClient::ILibrary>("libAntilatencyStorageClient.so");
    LOGI("Antilatency: Storage library was created");

    m_environmentSelectorLibrary = InterfaceContract::getLibraryInterface<Alt::Environment::Selector::ILibrary>("libAntilatencyAltEnvironmentSelector.so");
    LOGI("Antilatency: Environment Selector library was created");

    m_altTrackingLibrary = InterfaceContract::getLibraryInterface<Alt::Tracking::ILibrary>("libAntilatencyAltTracking.so");
    LOGI("Antilatency: AltTracking library was created");

    m_trackingAlignmentLibrary = InterfaceContract::getLibraryInterface<TrackingAlignment::ILibrary>("libAntilatencyTrackingAlignment.so");
    LOGI("Antilatency: AltTrackingAlignment library was created");

    JavaVM* jvm;
    env->GetJavaVM(&jvm);

    LOGI("Antilatency: JavaVM was taken");

    auto adnJni = m_adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    LOGI("Antilatency: adn wrapped interface was taken");

    if(jvm == nullptr)
        LOGI("Antilatency: jvm is nullptr");
    if(activity == nullptr)
        LOGI("Antilatency: activity is nullptr");
    if(adnJni == nullptr){
        LOGI("Antilatency: adnJni is nullptr");
    }
    try{
        adnJni.initJni(jvm, activity);
    }
    catch (Antilatency::InterfaceContract::Exception& ex) {
        LOGI("Antilatency: Antilatency::InterfaceContract::Exception");
        LOGI("Antilatency: %s", ex.what());
    }
    catch (...){
        LOGI("Antilatency: Any catcher");
    }

    LOGI("Antilatency: adn jni was initialized");

    auto antilatencyStorageClientJni = m_antilatencyStorageLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    LOGI("Antilatency: storageclient wrapped interface was taken");
    antilatencyStorageClientJni.initJni(jvm, activity);

    LOGI("Antilatency: Device Network ver.: %s", m_adnLibrary.getVersion().data());
    auto filter = m_adnLibrary.createFilter();
    LOGI("Antilatency: Filter was created");

    filter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    filter.addIpDevice(Antilatency::DeviceNetwork::Constants::AllIpDevicesIp, Antilatency::DeviceNetwork::Constants::AllIpDevicesMask);

    m_deviceNetwork = m_adnLibrary.createNetwork(filter);
    LOGI("Antilatency: network was created");

    auto environmentCode = m_antilatencyStorageLibrary.getLocalStorage().read("environment", "default");
    LOGI("Antilatency: environmentCode was read");

    m_environment = m_environmentSelectorLibrary.createEnvironment(environmentCode);
    LOGI("Antilatency: environment was created");

    //Get placement code from AltSystem app (default in this case) and create placement from it.
    auto placementCode = m_antilatencyStorageLibrary.getLocalStorage().read("placement", "default");
    LOGI("Antilatency: placementCode was read");

    m_placement = m_altTrackingLibrary.createPlacement(placementCode);
    LOGI("Antilatency: placement was read");

    m_trackingAlignment = m_trackingAlignmentLibrary.createTrackingAlignment(MathUtils::FloatFromQ(m_placement.rotation), m_extrapolationTime);
    LOGI("Antilatency: alignment was created");


    m_rigPose = Antilatency::Math::floatP3Q();
    m_startTime = std::chrono::high_resolution_clock::now();
}

AntilatencyManager::~AntilatencyManager() {
    m_trackingThreadRunning = false;
    if(m_trackingThread.joinable()) {
        m_trackingThread.join();
    }
}

void AntilatencyManager::doTracking() {
    //Get current Antilatency Device Network update id. It will be incremented every time any supported device is added or removed.
    auto _updateId = m_deviceNetwork.getUpdateId();
    LOGI("Antilatency: UpdateID is %u", _updateId);

    if(updateId != _updateId){
        updateId = _updateId;
        LOGALT("Antilatency Device Network update id has been incremented: %u", updateId);

        //Create tracking cotask constructor to check if node supports tracking and start tracking task on node.
        auto cotaskConstructor = m_altTrackingLibrary.createTrackingCotaskConstructor();

        //Get all currently connected nodes that supports tracking task.
        auto nodes = cotaskConstructor.findSupportedNodes(m_deviceNetwork);
        if(!nodes.empty())
            LOGI("Antilatency head node: %u", nodes[0]);
        for(auto node : nodes) {
            //Check if node is idle, we cannot start task on invalid nodes or on nodes that already has task started on it.
            if(m_deviceNetwork.nodeGetStatus(node) == Antilatency::DeviceNetwork::NodeStatus::Idle) {
                handleNode(node);
            }
        }
    }

    auto rmIter = std::remove_if(m_trackers.begin(), m_trackers.end(), [this](AntilatencyTracker& tracker) {
        if(tracker.cotask.isTaskFinished()) {
            LOGALT("Tracking node offline: %s", tracker.serialNumber.data());
            return true;
        } else {
            updateTracker(tracker);
            return false;
        }
    });
    m_trackers.erase(rmIter, m_trackers.end());

}

void AntilatencyManager::startTrackingAlignment(){
    if(m_trackingAlignment){
        stopTrackingAlignment();
    }

    m_trackingAlignment = m_trackingAlignmentLibrary.createTrackingAlignment(MathUtils::FloatFromQ(m_placement.rotation), m_extrapolationTime);
    m_altInitialPositionApplied = false;
}

void AntilatencyManager::stopTrackingAlignment(){
    if(!m_trackingAlignment){
        return;
    }

    m_trackingAlignment = nullptr;
}

void AntilatencyManager::handleNode(Antilatency::DeviceNetwork::NodeHandle node) {
    uint8_t trackerType = AntilatencyTracker::TYPE_UNKNOWN;
    try {
        std::string serialNumber = m_deviceNetwork.nodeGetStringProperty(node, DeviceNetwork::Interop::Constants::HardwareSerialNumberKey);
        std::string tag = m_deviceNetwork.nodeGetStringProperty(m_deviceNetwork.nodeGetParent(node), "Tag");
        if(tag == "HMD") {
            trackerType = AntilatencyTracker::TYPE_HMD;
        } else if(tag == "LeftHand") {
            trackerType = AntilatencyTracker::TYPE_LEFT_CONTROLLER;
        } else if(tag == "RightHand") {
            trackerType = AntilatencyTracker::TYPE_RIGHT_CONTROLLER;
        } else {
            LOGALTE("Unknown tag: %s", tag.data());
            return;
        }

        auto cotaskConstructor = m_altTrackingLibrary.createTrackingCotaskConstructor();
        auto trackingCotask = cotaskConstructor.startTask(m_deviceNetwork, node, m_environment);
        LOGI("Antilatency: cotask was started");
        m_trackers.push_back(AntilatencyTracker{ trackerType, serialNumber, trackingCotask });
        LOGALT("Tracking node online [%s]: %s", tag.data(), serialNumber.data());

    } catch (InterfaceContract::Exception &e) {
        LOGALTE("Handle node failed: %s", e.message().data());
    }
}


void AntilatencyManager::updateTracker(AntilatencyTracker &tracker) {
    if(tracker.cotask.isTaskFinished()) {
        return;
    }
    switch (tracker.type) {
        case AntilatencyTracker::TYPE_HMD:
            m_antilatencyTrackingData.head = proceedTrackingAlignment(tracker);
            break;
        case AntilatencyTracker::TYPE_LEFT_CONTROLLER:
            m_antilatencyTrackingData.leftHand = tracker.cotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
            break;
        case AntilatencyTracker::TYPE_RIGHT_CONTROLLER:
            m_antilatencyTrackingData.rightHand = tracker.cotask.getState(Antilatency::Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);
            break;
        default:
            break;
    }
}

Antilatency::Alt::Tracking::State AntilatencyManager::proceedTrackingAlignment(AntilatencyTracker &tracker){

    if(!tracker.cotask){
        return Antilatency::Alt::Tracking::State();
    }

    auto trackingState = tracker.cotask.getState(Alt::Tracking::Constants::DefaultAngularVelocityAvgTime);

    auto resultState = Antilatency::Alt::Tracking::State();

    if(!m_trackingAlignment){
        return Antilatency::Alt::Tracking::State();
    }

    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization){
        return Antilatency::Alt::Tracking::State();
    }

    if(trackingState.stability.stage == Alt::Tracking::Stage::Tracking6Dof && trackingState.stability.value > 0.075f){
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> timeSinceStart = currentTime - m_startTime;



        auto dTrackingState = MathUtils::FloatFromQ(trackingState.pose.rotation);
        auto dMRigRotation = MathUtils::FloatFromQ(m_rigPose.rotation);

        auto result = m_trackingAlignment.update(
                dTrackingState,
                dMRigRotation,
                timeSinceStart.count()
        );
        externalSpace = result;

        m_extrapolationTime = static_cast<float>(result.timeBAheadOfA);
        m_placement.rotation = MathUtils::FloatFromQ(result.rotationARelativeToB);

        trackingSpaceRotation = MathUtils::FloatFromQ(result.rotationBSpace);
    }

    trackingState = tracker.cotask.getExtrapolatedState(m_placement, m_extrapolationTime);

    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization){
        return Antilatency::Alt::Tracking::State();
    }


    LOGI("Antilatency:: trackingSpaceRotation = (%f,%f,%f, %f)", trackingSpaceRotation.x, trackingSpaceRotation.y,trackingSpaceRotation.z,trackingSpaceRotation.w);
    auto resultRotation = MathUtils::MultiplyQuat(trackingSpaceRotation, m_rigPose.rotation);

    resultState.pose.rotation = resultRotation;

//    if(trackingState.stability.stage == Antilatency::Alt::Tracking::Stage::Tracking6Dof){
//        auto a = trackingState.pose.position;
//        auto bSpace = rot
//    }

    resultState.pose.position = trackingState.pose.position;

    lastHMDPosition =  resultState.pose.position;

    return resultState;
}

Antilatency::Math::floatP3Q AntilatencyManager::getPlacement(){
    return m_placement;
}

std::optional<Antilatency::TrackingAlignment::State> AntilatencyManager::getExternalSpace(){
    return externalSpace;
}

void AntilatencyManager::controllerRotationCorrection(Antilatency::Math::floatQ BControllerOrientation, Antilatency::Math::floatQ& correctionRotationResult){

    BControllerOrientation.z *= -1.0f;
    BControllerOrientation.w *= -1.0f;

    if(externalSpace.has_value())
        correctionRotationResult = MathUtils::MultiplyQuat(externalSpace.value().rotationBSpace, BControllerOrientation);
    else
        correctionRotationResult = BControllerOrientation;

    correctionRotationResult.z *= -1.0f;
    correctionRotationResult.w *= -1.0f;
}

const Antilatency::Math::float3 AntilatencyManager::controllerVelocityCorrection(const Antilatency::Math::float3& vectorForCorrection) {
    Antilatency::Math::float3 result = {0,0,0};

    LOGI("ControllerVectorCorrection was called");

    if(externalSpace.has_value() && (MathUtils::ToGLMVec3(vectorForCorrection) != MathUtils::ToGLMVec3(Antilatency::Math::float3{0,0,0}))){
        auto glmVectorBSpacePosition = MathUtils::ToGLMVec3(vectorForCorrection);
        glmVectorBSpacePosition.z *= -1;

        auto glmRotationBSpace = MathUtils::ToGLMQuat(externalSpace.value().rotationBSpace);

        result = MathUtils::Float3FromPosition(glm::rotate(glmRotationBSpace, glmVectorBSpacePosition));

        result.z *= -1.0f;
    }
    else{
        LOGI("Antilatency: Error, externalSpace hasn't value");
    }

    return result;
}

const Antilatency::Math::float3 AntilatencyManager::controllerPositionCorrection(const Antilatency::Math::float3& vectorForCorrection, int controllerID){

    Antilatency::Math::float3 result = {0,0,0};

    if(externalSpace.has_value() && (MathUtils::ToGLMVec3(vectorForCorrection) != MathUtils::ToGLMVec3(Antilatency::Math::float3{0,0,0}))){
        auto glmVectorBSpacePosition = MathUtils::ToGLMVec3(vectorForCorrection);
        glmVectorBSpacePosition.z *= -1;

        auto glmRotationBSpace = MathUtils::ToGLMQuat(externalSpace.value().rotationBSpace);
        auto lastHMDBPosition = lastHMDOwnPosition;

        lastHMDBPosition.z *= -1;
        positionOffset = MathUtils::Float3FromPosition(glm::rotate(glmRotationBSpace, glmVectorBSpacePosition - MathUtils::ToGLMVec3(lastHMDBPosition)));
        result = MathUtils::Float3FromPosition(MathUtils::ToGLMVec3(lastHMDPosition) + MathUtils::ToGLMVec3(positionOffset));

        result.z *= -1.0f;

        lastControllerPositionASpace[controllerID] = result;
    }
    else{
        LOGI("Antilatency: Error, externalSpace hasn't value");
    }

    return result;
}

Antilatency::Math::float3 AntilatencyManager::getLastControllerPosition(int controllerID) const{
    return lastControllerPositionASpace[controllerID];
}
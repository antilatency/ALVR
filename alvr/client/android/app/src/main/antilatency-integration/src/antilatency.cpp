#include "antilatency.h"
#include <unistd.h>
#include <sstream>
#include <memory>
#include <chrono>
#include <antilatency/Antilatency.InterfaceContract.LibraryLoader.h>
#define LOGALT(...) __android_log_print(ANDROID_LOG_DEBUG, "Cloud VR Antilatency", __VA_ARGS__)
#define LOGALTI(...) __android_log_print(ANDROID_LOG_INFO, "Cloud VR Antilatency", __VA_ARGS__)
#define LOGALTE(...) __android_log_print(ANDROID_LOG_ERROR, "Cloud VR Antilatency", __VA_ARGS__)
// Antilatency数据结构

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
    m_antilatencyStorageLibrary = InterfaceContract::getLibraryInterface<StorageClient::ILibrary>("libAntilatencyStorageClient.so");
    m_environmentSelectorLibrary = InterfaceContract::getLibraryInterface<Alt::Environment::Selector::ILibrary>("libAntilatencyAltEnvironmentSelector.so");
    m_altTrackingLibrary = InterfaceContract::getLibraryInterface<Alt::Tracking::ILibrary>("libAntilatencyAltTracking.so");
    m_trackingAlignmentLibrary = InterfaceContract::getLibraryInterface<TrackingAlignment::ILibrary>("libAntilatencyTrackingAlignment.so");
//
    JavaVM* jvm;
    env->GetJavaVM(&jvm);

//    initJni(m_adnLibrary, jvm, activity);
//    initJni(m_altTrackingLibrary, jvm, activity);
//    initJni(m_environmentSelectorLibrary, jvm, activity);
//    initJni(m_antilatencyStorageLibrary, jvm, activity);
//    initJni(m_trackingAlignmentLibrary, jvm, activity);
//
    auto adnJni = m_adnLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    adnJni.initJni(jvm, activity);
    auto antilatencyStorageClientJni = m_antilatencyStorageLibrary.queryInterface<AndroidJniWrapper::IAndroidJni>();
    antilatencyStorageClientJni.initJni(jvm, activity);
//
    //Set log verbosity level for Antilatency Device Network library.
    m_adnLibrary.setLogLevel(Antilatency::DeviceNetwork::LogLevel::Trace);
    //m_adnLibrary.setLogLevel(Antilatency::DeviceNetwork::LogLevel::Trace);

    LOGI("Antilatency Device Network ver.: %s", m_adnLibrary.getVersion().data());
    auto filter = m_adnLibrary.createFilter();

    //Alt socket USB device ID
    Antilatency::DeviceNetwork::UsbDeviceFilter antilatencyUsbDeviceType;
    antilatencyUsbDeviceType.pid = 0x0000;
    antilatencyUsbDeviceType.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;
//    Antilatency::DeviceNetwork::UsbDeviceFilter antilatencyUsbDeviceType;
//    antilatencyUsbDeviceType.pid = 0x0000;
//    antilatencyUsbDeviceType.vid = Antilatency::DeviceNetwork::UsbVendorId::Antilatency;

    filter.addUsbDevice(antilatencyUsbDeviceType);
    filter.addUsbDevice(Antilatency::DeviceNetwork::Constants::AllUsbDevices);
    filter.addIpDevice(Antilatency::DeviceNetwork::Constants::AllIpDevicesIp, Antilatency::DeviceNetwork::Constants::AllIpDevicesMask);

    m_deviceNetwork = m_adnLibrary.createNetwork(filter);

    auto environmentCode = m_antilatencyStorageLibrary.getLocalStorage().read("environment", "default");
    m_environment = m_environmentSelectorLibrary.createEnvironment(environmentCode);
    //Get placement code from AltSystem app (default in this case) and create placement from it.
    auto placementCode = m_antilatencyStorageLibrary.getLocalStorage().read("placement", "OculusQuest");
    m_placement = m_altTrackingLibrary.createPlacement(placementCode);
    m_trackingAlignment = m_trackingAlignmentLibrary.createTrackingAlignment(MathUtils::FloatFromQ(m_placement.rotation), m_extrapolationTime);
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
        if(updateId != _updateId){
            updateId = _updateId;
            LOGALT("Antilatency Device Network update id has been incremented: %u", updateId);
            LOGALT("Searching for tracking nodes...");

            //Create tracking cotask constructor to check if node supports tracking and start tracking task on node.
            auto cotaskConstructor = m_altTrackingLibrary.createTrackingCotaskConstructor();

            //Get all currently connected nodes that supports tracking task.
            auto nodes = cotaskConstructor.findSupportedNodes(m_deviceNetwork);

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
        m_extrapolationTime = static_cast<float>(result.timeBAheadOfA);
        m_placement.rotation = MathUtils::FloatFromQ(result.rotationARelativeToB);
        trackingSpaceRotation = MathUtils::FloatFromQ(result.rotationBSpace);
    }
    trackingState = tracker.cotask.getExtrapolatedState(m_placement, m_extrapolationTime);
    if (trackingState.stability.stage == Alt::Tracking::Stage::InertialDataInitialization){
        return Antilatency::Alt::Tracking::State();
    }
    if(trackingState.stability.stage == Alt::Tracking::Stage::Tracking6Dof){
        auto a = trackingState.pose.position;
        auto b = MathUtils::AddVec3(m_rigPose.position, trackingSpacePosition);
        Antilatency::Math::float3 averagePositionInASpace;
        if(!m_altInitialPositionApplied){
            averagePositionInASpace = a;
            m_altInitialPositionApplied = true;
        } else {
            auto aGLM = MathUtils::ToGLMVec3(a);
            auto bGLM = MathUtils::ToGLMVec3(b);
            auto averagePosition = (bGLM * m_bQuality + aGLM * trackingState.stability.value) / (trackingState.stability.value + m_bQuality);
            averagePositionInASpace = MathUtils::FromGLMToAntilatencyFloat3(averagePosition);
        }
        auto tspGLM = MathUtils::ToGLMVec3(averagePositionInASpace);
        tspGLM -= MathUtils::ToGLMVec3(b);
        tspGLM += MathUtils::ToGLMVec3(trackingSpacePosition);
        trackingSpacePosition = MathUtils::FromGLMToAntilatencyFloat3(tspGLM);
    }
    
    auto resultRotation = MathUtils::MultiplyQuat(trackingSpaceRotation, m_rigPose.rotation);
    resultState.pose.rotation = resultRotation;
    resultState.pose.position = trackingState.pose.position;
    return resultState;
}
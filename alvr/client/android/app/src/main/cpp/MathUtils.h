#pragma once

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

struct MathUtils {


    template<typename T>
    static Antilatency::Math::float3 Float3FromPosition(T position){
        Antilatency::Math::float3 result = {
            .x = position.x,
            .y = position.y,
            .z = position.z
        };

        return result;
    };

    static Antilatency::Math::floatQ MultiplyQuat(Antilatency::Math::floatQ q1, Antilatency::Math::floatQ q2) {
        Antilatency::Math::floatQ result;
        result.x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        result.z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
        result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;

        return result;
    }

    static Antilatency::Math::float3 AddVec3(Antilatency::Math::float3 v1, Antilatency::Math::float3 v2) {
        Antilatency::Math::float3 result;
        result.x = v1.x + v2.x;
        result.y = v1.y + v2.y;
        result.z = v1.z + v2.z;

        return result;
    }

    static glm::vec3 ToGLMVec3(Antilatency::Math::float3 a){
        auto result = glm::vec3(a.x, a.y, a.z);
        return result;
    }

    static Antilatency::Math::float3 FromGLMToAntilatencyFloat3(glm::vec3 a){
        Antilatency::Math::float3 result = {a.x, a.y, a.z};
        return result;
    }

    template<typename T>
    static Antilatency::Math::floatQ FloatFromQ(T quat){
        Antilatency::Math::floatQ result = {
            .x = static_cast<float>(quat.x),
            .y = static_cast<float>(quat.y),
            .z = static_cast<float>(quat.z),
            .w = static_cast<float>(quat.w)
        };

        return result;
    };

    template<typename T>
    static glm::quat ToGLMQuat(T quat){
        return glm::quat(static_cast<float>(quat.w), static_cast<float>(quat.x), static_cast<float>(quat.y), static_cast<float>(quat.z));
    }
};
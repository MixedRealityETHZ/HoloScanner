#pragma once
#include "HL2Interactions.g.h"
#include <thread>
#include <DirectXMath.h>
#include <winrt/Windows.Perception.Spatial.h>
#include <winrt/Windows.Perception.People.h>
#include <winrt/Windows.UI.Input.Spatial.h>

namespace winrt::HL2UnityPlugin::implementation
{
    struct HandJoint
    {
        DirectX::XMVECTOR position;
        DirectX::XMVECTOR orientation;
        DirectX::XMMATRIX transformation;  // TrackedHands::GetOrientedJoint()
        float radius;
        bool tracked;
    };

    struct Hand
    {
        DirectX::XMVECTOR position;
        DirectX::XMVECTOR orientation;	// Quaternion

        std::vector<HandJoint> handJoints;
    };

    struct HL2Interactions : HL2InteractionsT<HL2Interactions>
    {
        HL2Interactions();

        void SetReferenceCoordinateSystem(Windows::Perception::Spatial::SpatialCoordinateSystem refCoord);

        void Update(Windows::Perception::PerceptionTimestamp timestamp);

        Windows::Foundation::Numerics::float4x4 GetHeadTransform();

        bool IsHandTracked(HL2UnityPlugin::HandIndex handIndex);
        bool IsJointTracked(HL2UnityPlugin::HandIndex handIndex, HL2UnityPlugin::HandJointIndex jointIndex);
        Windows::Foundation::Numerics::float4x4 GetOrientedJoint(HL2UnityPlugin::HandIndex handIndex, HL2UnityPlugin::HandJointIndex jointIndex);

        void EnableEyeTracking();
        bool IsEyeTrackingEnabled();
        bool IsEyeTrackingActive();
        Windows::Foundation::Numerics::float4 GetEyeGazeOrigin();
        Windows::Foundation::Numerics::float4 GetEyeGazeDirection();

    private:

        DirectX::XMVECTOR m_headPosition;
        DirectX::XMVECTOR m_headForwardDirection;
        DirectX::XMVECTOR m_headUpDirection;
        DirectX::XMMATRIX m_headTransform;

        bool m_isArticulatedHandTrackingAPIAvailable;	// True if articulated hand tracking API is available
        bool m_handTracked[(size_t) HL2UnityPlugin::HandIndex::Count];
        Hand m_hand[(size_t) HL2UnityPlugin::HandIndex::Count];

        bool m_isEyeTrackingAvailable;	// True if system supports eye tracking APIs and an eye tracking system is available
        bool m_isEyeTrackingRequested;	// True if app requested to enable eye tracking
        bool m_isEyeTrackingEnabled;	// True if eye tracking was successfully enabled
        bool m_isEyeTrackingActive;		// True if eye tracking is actively tracking (calibration available, etc)
        DirectX::XMVECTOR m_eyeGazeOrigin;
        DirectX::XMVECTOR m_eyeGazeDirection;

        Windows::UI::Input::Spatial::SpatialInteractionManager m_spatialInteractionManager = nullptr;
        Windows::Perception::Spatial::SpatialCoordinateSystem m_refFrame = nullptr;

        static Windows::Foundation::Numerics::float4 XMVECTORToFloat4(const DirectX::XMVECTOR &vector);
        static Windows::Foundation::Numerics::float4x4 XMMATRIXToFloat4x4(const DirectX::XMMATRIX &matrix);
    };
}
namespace winrt::HL2UnityPlugin::factory_implementation
{
    struct HL2Interactions : HL2InteractionsT<HL2Interactions, implementation::HL2Interactions>
    {
    };
}

#include "footbot_diffusion.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>

CFootBotDiffusion::CFootBotDiffusion() :
    m_pcWheels(nullptr),
    m_pcProximity(nullptr),
    m_fWheelVelocity(2.5f),
    m_fRepulsiveGain(20.0) {} 

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");

    GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
    GetNodeAttributeOrDefault(t_node, "repulsive_gain", m_fRepulsiveGain, m_fRepulsiveGain);
}

void CFootBotDiffusion::ControlStep() {
    // Get readings from proximity sensor
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

    // Compute a repulsive force based on proximity readings
    CVector2 cRepulsiveForce;
    bool ObstacleDetected = false;

    for(size_t i = 0; i < tProxReads.size(); ++i) {
        if (tProxReads[i].Value > 0.0) {
            // Compute repulsive force as a vector away from the obstacle
            cRepulsiveForce += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
            ObstacleDetected = true;
        }
    }

    // Scale the repulsive force
    cRepulsiveForce *= m_fRepulsiveGain;

    // If an obstacle is detected, compute the resulting velocity vector
    if (ObstacleDetected) {
        // Compute the resulting velocity vector
        CVector2 cResultantVelocity = cRepulsiveForce;

        // Normalize the velocity vector
        if (cResultantVelocity.Length() > 0.0) {
            cResultantVelocity.Normalize();
        }

        // Set wheel velocities to move in the normalized direction
        m_pcWheels->SetLinearVelocity(-1 * m_fWheelVelocity * cResultantVelocity.GetX(),
                                      -1 * m_fWheelVelocity * cResultantVelocity.GetY());
    } else {
        // If no obstacle is detected, move forward by default
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
}

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

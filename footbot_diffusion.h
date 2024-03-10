#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#include <argos3/core/utility/math/rng.h>

using namespace argos;

class CFootBotDiffusion : public CCI_Controller {

public:
  CFootBotDiffusion();
  virtual ~CFootBotDiffusion() {}
  virtual void Init(TConfigurationNode& t_node);
  virtual void ControlStep();
  virtual void Reset(){}
  virtual void Destroy() {}

private:
  CCI_DifferentialSteeringActuator* m_pcWheels;  
  CCI_FootBotProximitySensor* m_pcProximity;     

  CDegrees m_cAlpha;
  Real m_fDelta;
  Real m_fWheelVelocity;
  CRange<CRadians> m_cGoStraightAngleRange;

  Real m_fRepulsiveGain;  // Declared member variable

};

#endif

/* Include the controller definition */
#include "mah_controller.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcPosSens(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.1f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)),
   cGoal(CVector2(0,0)) {}
   

/****************************************/
/****************************************/



void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    \
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcPosSens = GetSensor<CCI_PositioningSensor>("positioning");
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   idx = 0;
   steps = 0;
   prev_pos = m_pcPosSens->GetReading().Position;
   /*cSocket.Connect("localhost",12345);*/
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   GetNodeAttributeOrDefault(t_node, "id", id, id);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
/*   if(id==1){
   	y_coord = {-1.5, -1.48, -1.44, -1.39, -1.32, -1.25, -1.18, -1.11, -1.04, -0.97, -0.9, -0.83, -0.76, -0.69, -0.62, -0.55, -0.48, -0.41, -0.34, -0.27, -0.2, -0.13, -0.06, 0.01, 0.08, 0.15, 0.22, 0.29, 0.36, 0.43, 0.5, 0.57, 0.64, 0.71, 0.78, 0.85, 0.92, 0.99, 1.04, 1.09, 1.14, 1.19,1.5};
   	x_coord = {-1.5, -1.41, -1.32, -1.24, -1.18, -1.12, -1.06, -1.0, -0.94, -0.88, -0.82, -0.76, -0.7, -0.64, -0.58, -0.52, -0.46, -0.4, -0.34, -0.28, -0.22, -0.16, -0.1, -0.04, 0.02, 0.08, 0.14, 0.2, 0.26, 0.32, 0.38, 0.44, 0.5, 0.56, 0.62, 0.68, 0.74, 0.8, 0.88, 0.96, 1.04, 1.12,1.5};
   }
   else if(id==2){
   	y_coord = {1.5, 1.46, 1.39, 1.3, 1.21, 1.13, 1.06, 0.99, 0.92, 0.85, 0.78, 0.71, 0.64, 0.57, 0.5, 0.43, 0.36, 0.29, 0.22, 0.15, 0.08, 0.01, -0.06, -0.13, -0.18, -0.23, -0.28, -0.33, -0.38, -0.43, -0.48, -0.53, -0.58, -0.63, -0.68, -0.73, -0.78, -0.83, -0.88, -0.93, -0.98, -1.03, -1.08, -1.13, -1.18,-1.5};
   	x_coord = {1.5, 1.59, 1.65, 1.68, 1.67, 1.62, 1.56, 1.5, 1.44, 1.38, 1.32, 1.26, 1.2, 1.14, 1.08, 1.02, 0.96, 0.9, 0.84, 0.78, 0.72, 0.66, 0.6, 0.54, 0.46, 0.38, 0.3, 0.22, 0.14, 0.06, -0.02, -0.1, -0.18, -0.26, -0.34, -0.42, -0.5, -0.58, -0.66, -0.74, -0.82, -0.9, -0.98, -1.06, -1.14,-1.5};
   }
   else{
   	y_coord = {-1.5, -1.48, -1.44, -1.39, -1.32, -1.24, -1.15, -1.06, -0.97, -0.88, -0.8, -0.73, -0.68, -0.63, -0.58, -0.53, -0.48, -0.43, -0.38, -0.33, -0.28, -0.23, -0.18, -0.13, -0.08, -0.03, 0.02, 0.07, 0.12, 0.17, 0.22, 0.27, 0.32, 0.37, 0.42, 0.47, 0.52, 0.57, 0.62, 0.67, 0.72, 0.77, 0.82, 0.87, 0.92, 0.97, 1.04, 1.11, 1.18,1.5};
   	x_coord = {1.5, 1.59, 1.68, 1.76, 1.82, 1.87, 1.9, 1.91, 1.9, 1.87, 1.83, 1.77, 1.69, 1.61, 1.53, 1.45, 1.37, 1.29, 1.21, 1.13, 1.05, 0.97, 0.89, 0.81, 0.73, 0.65, 0.57, 0.49, 0.41, 0.33, 0.25, 0.17, 0.09, 0.01, -0.07, -0.15, -0.23, -0.31, -0.39, -0.47, -0.55, -0.63, -0.71, -0.79, -0.87, -0.95, -1.01, -1.07, -1.13,-1.5};
   }
}
*/
if(id==1){
    y_coord = {-1.5, -1.5, -1.4, -1.3, -1.2, -1.1, -1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.0, 0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,  0.8,  0.9,  1.0,   1.1,  1.2, 1.3, 1.4,1.5};
    x_coord = {-1.4, -1.3, -1.2, -1.1, -1.0,  -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.3, 0.3, 0.4, 0.5, 0.5, 0.6, 0.7, 0.7, 0.8, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.8, 1.8, 1.8, 1.8, 1.7, 1.6,1.5};
}
else if(id==2){
     y_coord = {1.5, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7, -1.8, -1.8, -1.8, -1.8, -1.7, -1.6, -1.6, -1.6, -1.6, -1.6,-1.5};
     x_coord = {1.6, 1.7, 1.8, 1.8, 1.8, 1.8, 1.7, 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.6, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1, -1.2, -1.3, -1.4, -1.4, -1.4, -1.4, -1.4,-1.5};
}
else{
    x_coord = {-1.4, -1.3, -1.2, -1.1, -1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.0, 0.1, 0.2, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 1.9, 1.9, 1.9, 1.9, 1.8, 1.7, 1.6, 1.5};
    y_coord = {1.5, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9, -1.0, -1.1, -1.2, -1.3, -1.4, -1.5};
}
}
/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {
   /* Get readings from proximity sensor */
   /* Sum them together */
   if(idx<std::size(x_coord)){
   	cGoal = CVector2(x_coord[idx],y_coord[idx]);
   }
   CVector3 cRobotPos3 = m_pcPosSens->GetReading().Position;
   CVector3 curr_pos = cRobotPos3;
   // CRay3 rays = CRay3(curr_pos,prev_pos);
   // DrawRay(rays,CColor::RED,1.0f);
   prev_pos = curr_pos;
   CVector2 cRobotPos(cRobotPos3.GetX(),cRobotPos3.GetY());
   CQuaternion cRobotQuat = m_pcPosSens->GetReading().Orientation;   
   CRadians cRobotAngle,a1,a2;
   cRobotQuat.ToEulerAngles(cRobotAngle,a1,a2);
   CVector2 cAccumulator = cRobotPos - cGoal;
   CRadians cAngle = cAccumulator.Angle() - cRobotAngle;
   double pi = 3.1416;
   CByteArray msg;
   std::string msg_string = GetId();
   for(int i=0;i<3;i++){
   	msg<<msg_string[i];
   }
   std::cout<<id<<" "<<cGoal<<std::endl;
   /*cSocket.SendByteArray(msg);*/
   while(cAngle.GetValue() > pi){
   	cAngle.SetValue(cAngle.GetValue() - 2*pi);
   }
   while(cAngle.GetValue() < -1*pi){
   	cAngle.SetValue(cAngle.GetValue() + 2*pi);
   }
   double distance = cAccumulator.Length();
   double speed = 0.1f;
   double delta_time = distance/speed;
   double delta_time_new = delta_time;
   if (cAngle.GetValue()>0.1){
   delta_time_new = delta_time*cAngle.GetValue()/sin(cAngle.GetValue());
   }
   double linear_vel = speed/(0.029112741f);
   double ang_vel = (0.085036758f*cAngle.GetValue()/delta_time)/(0.029112741f);
   double ang_vel_new = 2*(0.085036758f*cAngle.GetValue()/delta_time_new)/(0.029112741f);
   if(delta_time<1){
   linear_vel = 0;
   ang_vel_new = 0;
   idx += 1;
   if(idx<std::size(x_coord)){
   	CVector2 cnxtGoal = CVector2(x_coord[idx],y_coord[idx]);
   	CVector2 delta_dist = cGoal - cnxtGoal;
   	if(delta_dist.Length() <= 0.0001){
   		idx -= 1;
   		steps += 1;
   		linear_vel = 0;
   		ang_vel_new = 0;
   		if(steps>50){
   		 steps = 0;
   		 idx += 1;
   		}
   	}
   }
   }
   //std::cout<<delta_time<<" "<<ang_vel<<std::endl;
   //if(abs(cAngle.GetValue())< m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(linear_vel+ang_vel_new, linear_vel-ang_vel_new);
   //}
   //else {
      /* Turn, depending on the sign of the angle */
     // if(cAngle.GetValue() > 0.0f) {
         //m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
     // }
      //else {
        // m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      //}
  // }
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")

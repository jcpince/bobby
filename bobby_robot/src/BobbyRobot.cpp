#include <vector>

#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
#include <transmission_interface/transmission_interface.h>

using std::vector;
using namespace transmission_interface;

#define wrap_trans_state(state, curr_pos, curr_vel, curr_eff) \
  state.position.push_back(&curr_pos);  \
  state.velocity.push_back(&curr_vel);  \
  state.effort.push_back(&curr_eff);

#define wrap_trans_data(cmd_data, cmd_pos) \
  cmd_data.position.push_back(&cmd_pos);

class BobbyRobot
{
public:
  BobbyRobot()
   : sim_trans(-10.0,  // 10x reducer. Negative sign: actuator and joint spin in opposite directions
                 1.0), // joint position offset

     dif_trans(vector<double>(2, 5.0), // 5x reducer on each actuator
               vector<double>(2, 1.0)) // No reducer in joint output
  {
    // Wrap simple transmission raw data - current state
    wrap_trans_state(a_state_data[0], a_curr_pos[0], a_curr_vel[0],
        a_curr_eff[0]);
    wrap_trans_state(j_state_data[0], j_curr_pos[0], j_curr_vel[0],
        j_curr_eff[0]);

    // Wrap simple transmission raw data - position command
    wrap_trans_data(a_cmd_data[0], a_cmd_pos[0]);
    wrap_trans_data(j_cmd_data[0], j_cmd_pos[0]);

    // Wrap differential transmission raw data - current state
    wrap_trans_state(a_state_data[1], a_curr_pos[2], a_curr_vel[1], 
        a_curr_eff[1]);
    wrap_trans_state(a_state_data[1], a_curr_pos[2], a_curr_vel[2],
        a_curr_eff[2]);
    wrap_trans_state(j_state_data[1], j_curr_pos[2], j_curr_vel[1], 
        j_curr_eff[1]);
    wrap_trans_state(j_state_data[1], j_curr_pos[2], j_curr_vel[2], 
        j_curr_eff[2]);

    // Wrap differential transmission raw data - position command
    wrap_trans_data(a_cmd_data[1], a_cmd_pos[1]);
    wrap_trans_data(a_cmd_data[1], a_cmd_pos[2]);
    wrap_trans_data(j_cmd_data[1], j_cmd_pos[1]);
    wrap_trans_data(j_cmd_data[1], j_cmd_pos[2]);

    // ...once the raw data is wrapped, the rest is straightforward

    // Register transmissions to each interface
    act_to_jnt_state.registerHandle(
        ActuatorToJointStateHandle("sim_trans",
             &sim_trans,
             a_state_data[0],
             j_state_data[0]));

    act_to_jnt_state.registerHandle(
        ActuatorToJointStateHandle("dif_trans",
            &dif_trans,
            a_state_data[1],
            j_state_data[1]));

    jnt_to_act_pos.registerHandle(
        JointToActuatorPositionHandle("sim_trans",
            &sim_trans,
            a_cmd_data[0],
            j_cmd_data[0]));

    jnt_to_act_pos.registerHandle(
        JointToActuatorPositionHandle("dif_trans",
            &dif_trans,
            a_cmd_data[1],
            j_cmd_data[1]));

    // Names must be unique within a single transmission interface,
    // but a same name can be used in multiple interfaces,
    // as shown above
  }

  void read()
  {
    // Read actuator state from hardware
    std::cout << "Reading actuator state from HW" << std::endl;

    // Propagate current actuator state to joints
    act_to_jnt_state.propagate();
  }

  void write()
  {
    // Porpagate joint commands to actuators
    jnt_to_act_pos.propagate();

    // Send actuator command to hardware
    std::cout << "Sending actuator commands to HW" << std::endl;
  }

private:

  // Transmission interfaces
  ActuatorToJointStateInterface    act_to_jnt_state; // For propagating current actuator state to joint space
  JointToActuatorPositionInterface jnt_to_act_pos;   // For propagating joint position commands to actuator space

  // Transmissions
  SimpleTransmission       sim_trans;
  DifferentialTransmission dif_trans;

  // Actuator and joint space variables: wrappers around raw data
  ActuatorData  a_state_data[2]; // Size 2: One per transmission
  ActuatorData  a_cmd_data[2];
  JointData     j_state_data[2];
  JointData     j_cmd_data[2];

  // Actuator and joint space variables - raw data:
  // The first actuator/joint are coupled through a reducer.
  // The last two actuators/joints are coupled through a differential.
  double a_curr_pos[3]; // Size 3: One per actuator
  double a_curr_vel[3];
  double a_curr_eff[3];
  double a_cmd_pos[3];

  double j_curr_pos[3]; // Size 3: One per joint
  double j_curr_vel[3];
  double j_curr_eff[3];
  double j_cmd_pos[3];
};

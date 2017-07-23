#include "Pcv.h"
using namespace std;
#ifndef YAMLCPP_03
template < class _T >
        void operator >>(const YAML::Node& input, _T& value) {
                try {
                        value = input.as<_T>();
                        //input >> value;
                } catch (YAML::Exception &e) {
                       std::cout<<"Error converting from YAML! " << e.what()<<std::endl;
                }
        }
#endif
PCV::PCV(YAML::Node &doc, int step_freq ) : ctrl(NULL)
{  
  /////////////////////////
  /*   Parse Yaml Files */
  /////////////////////////
  Float num,den;
  string name;  
  Float temp;
  doc["name"] >> name;  
  doc["geometry"]["num_casters"] >> params.geometry.num_casters;
  for (int i = 0; i < params.geometry.num_casters; i++)
  {
    doc["geometry"]["caster_placement"][i][0] >> params.geometry.caster_placement[i].x;
    doc["geometry"]["caster_placement"][i][1] >> params.geometry.caster_placement[i].y;
    doc["geometry"]["caster_placement"][i][2] >> params.geometry.caster_placement[i].angle;
    doc["geometry"]["steering_offsets"][i] >> params.geometry.steering_offsets[i];
  }
  doc["kinematics"]["abs_max_linear_acceleration"] >> params.kinematics.abs_max_linear_acceleration;
  doc["kinematics"]["abs_max_linear_velocity"] >> params.kinematics.abs_max_linear_velocity;
  doc["kinematics"]["abs_max_rotation_acceleration"] >> params.kinematics.abs_max_rotation_acceleration;
  doc["kinematics"]["abs_max_rotation_velocity"] >> params.kinematics.abs_max_rotation_velocity;
  
  doc["caster"]["Nm_PER_AMP"] >> params.caster.Nm_PER_AMP;
  doc["caster"]["AMPS_PER_VOLT"] >> params.caster.AMPS_PER_VOLT;
  doc["caster"]["ENC_REV2CNT"] >> params.caster.ENC_REV2CNT;
  doc["caster"]["COUNTS_PER_VOLT"] >> params.caster.COUNTS_PER_VOLT;  
  doc["caster"]["b"] >> params.caster.b;
  doc["caster"]["r"] >> params.caster.r;
  doc["caster"]["f"] >> params.caster.f;
  doc["caster"]["Mf"] >> params.caster.Mf;
  doc["caster"]["If"] >> params.caster.If;
  doc["caster"]["Ih"] >> params.caster.Ih;
  doc["caster"]["Ii"] >> params.caster.Ii;
  doc["caster"]["Is"] >> params.caster.Is;
  doc["caster"]["It"] >> params.caster.It;
  doc["caster"]["Ij"] >> params.caster.Ij;
  doc["caster"]["px"] >> params.caster.px;
  doc["caster"]["py"] >> params.caster.py;
  doc["caster"]["Mp"] >> params.caster.Mp;
  doc["caster"]["Ip"] >> params.caster.Ip;
  doc["caster"]["b"] >> params.caster.b;
  doc["caster"]["Ns_num"] >> num;
  doc["caster"]["Ns_den"] >> den;
  params.caster.Ns = num/den;
  doc["caster"]["Nt_num"] >> num;
  doc["caster"]["Nt_den"] >> den;
  params.caster.Nt = num/den;
  doc["caster"]["Nw_num"] >> num;
  doc["caster"]["Nw_den"] >> den;
  params.caster.Nw = num/den;
  doc["gains"]["KPx_"] >> params.gains.KPx_;
  doc["gains"]["KPa_"] >> params.gains.KPa_;  
  doc["gains"]["KVx_"] >> params.gains.KVx_;
  doc["gains"]["KVa_"] >> params.gains.KVa_;
  doc["gains"]["KpE_"] >> params.gains.KpE_;
  doc["gains"]["KpC_"] >> params.gains.KpC_;
  doc["inertial"]["mass"] >> params.mass.mass;
  doc["inertial"]["inertia_flat"] >> params.mass.inertia_flat;
  doc["filter"]["FQD"] >> params.filter.FQD;
  doc["filter"]["FGXD"] >> params.filter.FGXD;
  doc["filter"]["FCXDD"] >> params.filter.FCXDD;
  doc["filter"]["FTE"] >> params.filter.FTE;
  doc["filter"]["FXD"] >> params.filter.FXD;  
  
  
  //MSG("Finished reading config files.\n");
  
  vector<Float> init_x(3,0.0);
    
  ctrl = new Ctrl(step_freq, init_x, params);
  if (ctrl==NULL)
    MSG("Error creating ctrl class.\n");
  ENC_REV2CNT = params.caster.ENC_REV2CNT;  
  
}

Pcv_Status PCV::Step(Pcv_Command pcv_cmd)
{
  return ctrl->Step(pcv_cmd);
}

void PCV::UpdateGoal(Pcv_Command pcv_cmd)
{
  PrVector destPos(3);    
  destPos[0] = pcv_cmd.traj_goal[0];
  destPos[1] = pcv_cmd.traj_goal[1];
  destPos[2] = pcv_cmd.traj_goal[2];
  ctrl->traj->dest2(destPos);
  //ctrl->TrajInit(pcv_cmd);
}

/*
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "m3/robots/humanoid_shm.h"
#include "m3rt/base/component_factory.h"


namespace m3 {

using namespace m3rt;
using namespace std;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
M3BaseStatus * M3HumanoidShm::GetBaseStatus() {
    return status.mutable_base();
    }

void  M3HumanoidShm::Startup() {
    M3CompShm::Startup();

    sds_status_size = sizeof ( M3HumanoidShmSdsStatus );
    sds_cmd_size = sizeof ( M3HumanoidShmSdsCommand );

    memset ( &status_to_sds, 0, sds_status_size );

    right_arm_extra_payload_mass_initial = bot->GetPayloadMass ( RIGHT_ARM );
    left_arm_extra_payload_mass_initial = bot->GetPayloadMass ( LEFT_ARM );

    for ( int i = 0; i < 3; i++ ) {
        right_arm_extra_payload_com_initial[i] = bot->GetPayloadInertia ( RIGHT_ARM,i );
        left_arm_extra_payload_com_initial[i] = bot->GetPayloadInertia ( LEFT_ARM,i );
        }

    }

void M3HumanoidShm::ResetCommandSds ( unsigned char * sds ) {

    memset ( sds,0,sizeof ( M3HumanoidShmSdsCommand ) );

    }


size_t M3HumanoidShm::GetStatusSdsSize() {
    return sds_status_size;
    }

size_t M3HumanoidShm::GetCommandSdsSize() {
    return sds_cmd_size;
    }

void M3HumanoidShm::SetCommandFromSds ( unsigned char * data ) {

    M3HumanoidShmSdsCommand * sds = ( M3HumanoidShmSdsCommand * ) data;
    request_command();
    memcpy ( &command_from_sds, sds, GetCommandSdsSize() );
    release_command();

    int64_t dt = GetBaseStatus()->timestamp()-command_from_sds.timestamp; // microseconds
    bool shm_timeout = ABS ( dt ) > ( timeout*1000 );

    if ( pwr != NULL ) {
        if ( shm_timeout )
            pwr->SetMotorEnable ( false );
        else
            pwr->SetMotorEnable ( true );
        }

    if ( bot != NULL ) {
        if ( shm_timeout )
            bot->SetMotorPowerOff();
        else
            bot->SetMotorPowerOn();

        for ( int i = 0; i < bot->GetNdof ( RIGHT_ARM ); i++ ) {
            if ( shm_timeout ) {
                bot->SetModeOff ( RIGHT_ARM, i );
                }
            else {
                bot->SetThetaDeg ( RIGHT_ARM, i, RAD2DEG ( command_from_sds.right_arm.q_desired[i] ) );
                bot->SetSlewRate ( RIGHT_ARM, i, RAD2DEG ( command_from_sds.right_arm.slew_rate_q_desired[i] ) );
                bot->SetThetaDotDeg ( RIGHT_ARM, i, RAD2DEG ( command_from_sds.right_arm.slew_rate_q_desired[i] ) );
                bot->SetTorque_mNm ( RIGHT_ARM, i, command_from_sds.right_arm.tq_desired[i] );
                bot->SetStiffness ( RIGHT_ARM, i, command_from_sds.right_arm.q_stiffness[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_right_arm()->set_ctrl_mode ( i, command_from_sds.right_arm.ctrl_mode[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_right_arm()->set_smoothing_mode ( i, command_from_sds.right_arm.smoothing_mode[i] );
                /*if (i == 0)
                {if (tmp_cnt++ == 1000)
                {
                  M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.right_arm.ctrl_mode[i]);
                  tmp_cnt = 0;
                }}*/
                //M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.right_arm.ctrl_mode[i]);
                }
            }

        // calc new COM for adjusted payload

        mReal com_new[3];
        mReal m_total = command_from_sds.right_arm.extra_payload_mass + right_arm_extra_payload_mass_initial;
        com_new[0] = ( command_from_sds.right_arm.extra_payload_com[0]*command_from_sds.right_arm.extra_payload_mass +
                       right_arm_extra_payload_com_initial[0]*right_arm_extra_payload_mass_initial ) /m_total;
        com_new[1] = ( command_from_sds.right_arm.extra_payload_com[1]*command_from_sds.right_arm.extra_payload_mass +
                       right_arm_extra_payload_com_initial[1]*right_arm_extra_payload_mass_initial ) /m_total;
        com_new[2] = ( command_from_sds.right_arm.extra_payload_com[2]*command_from_sds.right_arm.extra_payload_mass +
                       right_arm_extra_payload_com_initial[2]*right_arm_extra_payload_mass_initial ) /m_total;

        bot->SetPayloadMass ( RIGHT_ARM, m_total );

        for ( int i = 0; i < 3; i++ )
            bot->SetPayloadCom ( RIGHT_ARM, i, com_new[i] );

        for ( int i = 0; i < bot->GetNdof ( TORSO ); i++ ) {
            if ( shm_timeout ) {
                bot->SetModeOff ( TORSO, i );
                }
            else {
                bot->SetThetaDeg ( TORSO, i, RAD2DEG ( command_from_sds.torso.q_desired[i] ) );
                bot->SetSlewRate ( TORSO, i, RAD2DEG ( command_from_sds.torso.slew_rate_q_desired[i] ) );
                bot->SetThetaDotDeg ( TORSO, i, RAD2DEG ( command_from_sds.torso.slew_rate_q_desired[i] ) );
                bot->SetTorque_mNm ( TORSO, i, command_from_sds.torso.tq_desired[i] );
                bot->SetStiffness ( TORSO, i, command_from_sds.torso.q_stiffness[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_torso()->set_ctrl_mode ( i, command_from_sds.torso.ctrl_mode[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_torso()->set_smoothing_mode ( i, command_from_sds.torso.smoothing_mode[i] );
                //M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.right_arm.ctrl_mode[i]);
                }
            }

        for ( int i = 0; i < bot->GetNdof ( LEFT_ARM ); i++ ) {
            if ( shm_timeout ) {
                bot->SetModeOff ( LEFT_ARM, i );
                }
            else {
                bot->SetThetaDeg ( LEFT_ARM, i, RAD2DEG ( command_from_sds.left_arm.q_desired[i] ) );
                bot->SetSlewRate ( LEFT_ARM, i, RAD2DEG ( command_from_sds.left_arm.slew_rate_q_desired[i] ) );
                bot->SetThetaDotDeg ( LEFT_ARM, i, RAD2DEG ( command_from_sds.left_arm.slew_rate_q_desired[i] ) );
                bot->SetTorque_mNm ( LEFT_ARM, i, command_from_sds.left_arm.tq_desired[i] );
                bot->SetStiffness ( LEFT_ARM, i, command_from_sds.left_arm.q_stiffness[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_left_arm()->set_ctrl_mode ( i, command_from_sds.left_arm.ctrl_mode[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_left_arm()->set_smoothing_mode ( i, command_from_sds.left_arm.smoothing_mode[i] );
                //M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.right_arm.ctrl_mode[i]);
                }
            }

        m_total = command_from_sds.left_arm.extra_payload_mass + left_arm_extra_payload_mass_initial;
        com_new[0] = ( command_from_sds.left_arm.extra_payload_com[0]*command_from_sds.left_arm.extra_payload_mass +
                       left_arm_extra_payload_com_initial[0]*left_arm_extra_payload_mass_initial ) /m_total;
        com_new[1] = ( command_from_sds.left_arm.extra_payload_com[1]*command_from_sds.left_arm.extra_payload_mass +
                       left_arm_extra_payload_com_initial[1]*left_arm_extra_payload_mass_initial ) /m_total;
        com_new[2] = ( command_from_sds.left_arm.extra_payload_com[2]*command_from_sds.left_arm.extra_payload_mass +
                       left_arm_extra_payload_com_initial[2]*left_arm_extra_payload_mass_initial ) /m_total;

        bot->SetPayloadMass ( LEFT_ARM, m_total );

        for ( int i = 0; i < 3; i++ )
            bot->SetPayloadCom ( LEFT_ARM, i, com_new[i] );

        for ( int i = 0; i < bot->GetNdof ( HEAD ); i++ ) {
            if ( shm_timeout ) {
                bot->SetModeOff ( HEAD, i );
                }
            else {
                bot->SetThetaDeg ( HEAD, i, RAD2DEG ( command_from_sds.head.q_desired[i] ) );
                bot->SetSlewRate ( HEAD, i, RAD2DEG ( command_from_sds.head.slew_rate_q_desired[i] ) );
                bot->SetThetaDotDeg ( HEAD, i, RAD2DEG ( command_from_sds.head.slew_rate_q_desired[i] ) );
                bot->SetTorque_mNm ( HEAD, i, command_from_sds.head.tq_desired[i] );
                bot->SetStiffness ( HEAD, i, command_from_sds.head.q_stiffness[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_head()->set_ctrl_mode ( i, command_from_sds.head.ctrl_mode[i] );
                ( ( M3HumanoidCommand* ) bot->GetCommand() )->mutable_head()->set_smoothing_mode ( i, command_from_sds.head.smoothing_mode[i] );
                /*if (tmp_cnt++ == 1000)
                {
                  //M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.head.ctrl_mode[i]);
                  tmp_cnt = 0;
                }*/
                }
            }
        }

    if ( right_hand ) {

        for ( int i = 0; i < right_hand->GetNumDof(); i++ ) {
            if ( shm_timeout ) {
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_ctrl_mode ( i, JOINT_ARRAY_MODE_OFF );
                }
            else {
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_ctrl_mode ( i, command_from_sds.right_hand.ctrl_mode[i] );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_smoothing_mode ( i, command_from_sds.right_hand.smoothing_mode[i] );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_q_desired ( i, RAD2DEG ( command_from_sds.right_hand.q_desired[i] ) );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_q_slew_rate ( i, RAD2DEG ( command_from_sds.right_hand.slew_rate_q_desired[i] ) );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_qdot_desired ( i, RAD2DEG ( command_from_sds.right_hand.slew_rate_q_desired[i] ) );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_tq_desired ( i, command_from_sds.right_hand.tq_desired[i] );
                ( ( M3JointArrayCommand* ) right_hand->GetCommand() )->set_q_stiffness ( i, command_from_sds.right_hand.q_stiffness[i] );
                }
            /*	if (tmp_cnt++ == 200 && i == 0)
            	{
            		M3_DEBUG("mode %d : %d\n",i, (int)command_from_sds.right_hand.ctrl_mode[i]);
            		tmp_cnt = 0;
            	}*/
            }
        }

    if ( left_hand ) {
        for ( int i = 0; i < left_hand->GetNumDof(); i++ ) {
            if ( shm_timeout ) {
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_ctrl_mode ( i, JOINT_ARRAY_MODE_OFF );
                }
            else {
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_ctrl_mode ( i, command_from_sds.left_hand.ctrl_mode[i] );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_smoothing_mode ( i, command_from_sds.left_hand.smoothing_mode[i] );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_q_desired ( i, RAD2DEG ( command_from_sds.left_hand.q_desired[i] ) );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_q_slew_rate ( i, RAD2DEG ( command_from_sds.left_hand.slew_rate_q_desired[i] ) );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_qdot_desired ( i, RAD2DEG ( command_from_sds.left_hand.slew_rate_q_desired[i] ) );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_tq_desired ( i, command_from_sds.left_hand.tq_desired[i] );
                ( ( M3JointArrayCommand* ) left_hand->GetCommand() )->set_q_stiffness ( i, command_from_sds.left_hand.q_stiffness[i] );
                }
            }
        }

    if ( left_gripper ) {

        if ( shm_timeout ) {
            left_gripper->SetDesiredControlMode ( JOINT_MODE_OFF );
            }
        else {
            if ( command_from_sds.left_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_THETA )
                left_gripper->SetDesiredControlMode ( ( JOINT_MODE_THETA ) );
            else if ( command_from_sds.left_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_THETA_GC )
                left_gripper->SetDesiredControlMode ( ( JOINT_MODE_THETA_GC ) );
            else if ( command_from_sds.left_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_TORQUE )
                left_gripper->SetDesiredControlMode ( ( JOINT_MODE_TORQUE ) );
            else
                left_gripper->SetDesiredControlMode ( ( JOINT_MODE_OFF ) );

            /*if (tmp_cnt++ == 1000)
            {
              M3_DEBUG("m : %i\n", (int)command_from_sds.left_gripper.ctrl_mode[0]);
              M3_DEBUG("smoot : %i\n", (int)command_from_sds.left_gripper.smoothing_mode[0]);
              M3_DEBUG("th : %f\n", command_from_sds.left_gripper.q_desired[0]);
              M3_DEBUG("tq : %f\n", command_from_sds.left_gripper.tq_desired[0]);
              M3_DEBUG("tqd : %f\n", command_from_sds.left_gripper.slew_rate_q_desired[0]);
              tmp_cnt = 0;
            }*/
            left_gripper->SetDesiredThetaRad ( command_from_sds.left_gripper.q_desired[0] );
            left_gripper->SetDesiredStiffness ( command_from_sds.left_gripper.q_stiffness[0] );
            left_gripper->SetDesiredSmoothingMode ( ( SMOOTHING_MODE ) command_from_sds.left_gripper.smoothing_mode[0] );
            left_gripper->SetDesiredTorque ( command_from_sds.left_gripper.tq_desired[0] );
            left_gripper->SetDesiredThetaDotRad ( command_from_sds.left_gripper.slew_rate_q_desired[0] );
            left_gripper->SetSlewRate ( RAD2DEG ( command_from_sds.left_gripper.slew_rate_q_desired[0] ) );
            }
        }

    if ( right_gripper ) {

        if ( shm_timeout ) {
            right_gripper->SetDesiredControlMode ( JOINT_MODE_OFF );
            }
        else {
            if ( command_from_sds.right_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_THETA )
                right_gripper->SetDesiredControlMode ( ( JOINT_MODE_THETA ) );
            else if ( command_from_sds.right_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_THETA_GC )
                right_gripper->SetDesiredControlMode ( ( JOINT_MODE_THETA_GC ) );
            else if ( command_from_sds.right_gripper.ctrl_mode[0] == JOINT_ARRAY_MODE_TORQUE )
                right_gripper->SetDesiredControlMode ( ( JOINT_MODE_TORQUE ) );
            else
                right_gripper->SetDesiredControlMode ( ( JOINT_MODE_OFF ) );

            /*if (tmp_cnt++ == 1000)
            {
              M3_DEBUG("m : %i\n", (int)command_from_sds.left_gripper.ctrl_mode[0]);
              M3_DEBUG("smoot : %i\n", (int)command_from_sds.left_gripper.smoothing_mode[0]);
              M3_DEBUG("th : %f\n", command_from_sds.left_gripper.q_desired[0]);
              M3_DEBUG("tq : %f\n", command_from_sds.left_gripper.tq_desired[0]);
              M3_DEBUG("tqd : %f\n", command_from_sds.left_gripper.slew_rate_q_desired[0]);
              tmp_cnt = 0;
            }*/
            right_gripper->SetDesiredThetaRad ( command_from_sds.right_gripper.q_desired[0] );
            right_gripper->SetDesiredStiffness ( command_from_sds.right_gripper.q_stiffness[0] );
            right_gripper->SetDesiredSmoothingMode ( ( SMOOTHING_MODE ) command_from_sds.right_gripper.smoothing_mode[0] );
            right_gripper->SetDesiredTorque ( command_from_sds.right_gripper.tq_desired[0] );
            right_gripper->SetDesiredThetaDotRad ( command_from_sds.right_gripper.slew_rate_q_desired[0] );
            right_gripper->SetSlewRate ( RAD2DEG ( command_from_sds.right_gripper.slew_rate_q_desired[0] ) );
            }
        }

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void M3HumanoidShm::SetSdsFromStatus ( unsigned char * data ) {
    status_to_sds.timestamp = GetBaseStatus()->timestamp();

    if ( bot ) {
	    //memcpy(&status_to_sds.bot,bot,sizeof(M3Humanoid));
        for ( int i = 0; i < bot->GetNdof ( RIGHT_ARM ); i++ ) {
            status_to_sds.right_arm.theta[i] = bot->GetThetaDeg ( RIGHT_ARM,i );
            status_to_sds.right_arm.thetadot[i] = bot->GetThetaDotDeg ( RIGHT_ARM,i );
            status_to_sds.right_arm.torque[i] = bot->GetTorque_mNm ( RIGHT_ARM,i );
            status_to_sds.right_arm.gravity[i] = bot->GetGravity ( RIGHT_ARM,i );
            }
        if ( bot->GetNdof ( RIGHT_ARM ) ) {
            for ( int i = 0; i < 6; i++ )
                for ( int j = 0; j < bot->GetNdof ( RIGHT_ARM ); j++ )
                    status_to_sds.right_arm.jacobian[i][j] = bot->GetJacobian ( RIGHT_ARM ) ( i,j );
            for ( int i=0; i < 3; i++ )
                status_to_sds.right_arm.end_pos[i] = bot->GetEndPosition ( RIGHT_ARM ) [i];
            for ( int i=0; i < 3; i++ )
                for ( int j=0; j < 3; j++ )
                    status_to_sds.right_arm.end_rot[i][j] = bot->GetEndRotation ( RIGHT_ARM ) ( i,j );
            }

        for ( int i = 0; i < bot->GetNdof ( LEFT_ARM ); i++ ) {
            status_to_sds.left_arm.theta[i] = bot->GetThetaDeg ( LEFT_ARM,i );
            status_to_sds.left_arm.thetadot[i] = bot->GetThetaDotDeg ( LEFT_ARM,i );
            status_to_sds.left_arm.torque[i] = bot->GetTorque_mNm ( LEFT_ARM,i );
            status_to_sds.left_arm.gravity[i] = bot->GetGravity ( LEFT_ARM,i );
            }
        if ( bot->GetNdof ( LEFT_ARM ) ) {
            for ( int i = 0; i < 6; i++ )
                for ( int j = 0; j < bot->GetNdof ( LEFT_ARM ); j++ )
                    status_to_sds.left_arm.jacobian[i][j] = bot->GetJacobian ( LEFT_ARM ) ( i,j );
            for ( int i=0; i < 3; i++ )
                status_to_sds.left_arm.end_pos[i] = bot->GetEndPosition ( LEFT_ARM ) [i];
            for ( int i=0; i < 3; i++ )
                for ( int j=0; j < 3; j++ )
                    status_to_sds.left_arm.end_rot[i][j] = bot->GetEndRotation ( LEFT_ARM ) ( i,j );
            }

        for ( int i = 0; i < bot->GetNdof ( TORSO ); i++ ) {
            status_to_sds.torso.theta[i] = bot->GetThetaDeg ( TORSO,i );
            status_to_sds.torso.thetadot[i] = bot->GetThetaDotDeg ( TORSO,i );
            status_to_sds.torso.torque[i] = bot->GetTorque_mNm ( TORSO,i );
            status_to_sds.torso.gravity[i] = bot->GetGravity ( TORSO,i );
            }
        if ( bot->GetNdof ( TORSO ) ) {
            for ( int i = 0; i < 6; i++ )
                for ( int j=0; j<bot->GetNdof ( TORSO ); j++ )
                    status_to_sds.torso.jacobian[i][j] = bot->GetJacobian ( TORSO ) ( i,j );
            }
        for ( int i = 0; i < bot->GetNdof ( HEAD ); i++ ) {
            status_to_sds.head.theta[i] = bot->GetThetaDeg ( HEAD,i );
            status_to_sds.head.thetadot[i] = bot->GetThetaDotDeg ( HEAD,i );
            status_to_sds.head.torque[i] = bot->GetTorque_mNm ( HEAD,i );
            }
        }

    if ( right_hand ) {
        for ( int i = 0; i < right_hand->GetNumDof(); i++ ) {
            status_to_sds.right_hand.theta[i] = right_hand->GetThetaDeg ( i );
            status_to_sds.right_hand.thetadot[i] = right_hand->GetThetaDotDeg ( i );
            status_to_sds.right_hand.torque[i] = right_hand->GetTorque ( i );
            }
        }

    if ( left_hand ) {
        for ( int i = 0; i < left_hand->GetNumDof(); i++ ) {
            status_to_sds.left_hand.theta[i] = left_hand->GetThetaDeg ( i );
            status_to_sds.left_hand.thetadot[i] = left_hand->GetThetaDotDeg ( i );
            status_to_sds.left_hand.torque[i] = left_hand->GetTorque ( i );
            }
        }

    if ( left_gripper ) {
        status_to_sds.left_gripper.theta[0] = left_gripper->GetThetaDeg();
        status_to_sds.left_gripper.thetadot[0] = left_gripper->GetThetaDotDeg();
        status_to_sds.left_gripper.torque[0] = left_gripper->GetTorque();
        }

    if ( right_gripper ) {
        status_to_sds.right_gripper.theta[0] = right_gripper->GetThetaDeg();
        status_to_sds.right_gripper.thetadot[0] = right_gripper->GetThetaDotDeg();
        status_to_sds.right_gripper.torque[0] = right_gripper->GetTorque();
        }


    if ( right_loadx6 ) {
        for ( int i = 0; i < 6; i++ ) {
            status_to_sds.right_loadx6.wrench[i] = right_loadx6->GetWrench ( i );
            }
        }

    if ( left_loadx6 ) {
        for ( int i = 0; i < 6; i++ ) {
            status_to_sds.left_loadx6.wrench[i] = left_loadx6->GetWrench ( i );
            }
        }

    M3HumanoidShmSdsStatus * sds = ( M3HumanoidShmSdsStatus * ) data;
    request_status();
    memcpy ( sds, &status_to_sds, GetStatusSdsSize() );
    release_status();

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool M3HumanoidShm::LinkDependentComponents() {
    tmp_cnt = 0;

    if ( bot_name.size() !=0 ) {
		bot= dynamic_cast< M3Humanoid* > (factory->GetComponent ( bot_name ));
        if ( bot==NULL ) {
            M3_ERR ( "M3Humanoid component %s declared for M3BotShm but could not be linked\n",
                     bot_name.c_str() );
            //return false;
            }
        }

    if ( pwr_name.size() !=0 ) {
		pwr= dynamic_cast< M3Pwr* >( factory->GetComponent ( pwr_name ));
        if ( pwr==NULL )
            M3_WARN ( "M3Pwr component %s declared for M3BotShm but could not be linked\n",
                      pwr_name.c_str() );

        }

    if ( right_hand_name.size() !=0 ) {
		right_hand= dynamic_cast< M3Hand* >( factory->GetComponent ( right_hand_name )); //May be null if not on this robot model
        if ( right_hand==NULL ) {
            M3_WARN ( "M3Hand component %s declared for M3BotShm but could not be linked\n",
                      right_hand_name.c_str() );
            //return false;
            }
        }

    if ( right_loadx6_name.size() !=0 ) {
		right_loadx6= dynamic_cast< M3LoadX6* >( factory->GetComponent ( right_loadx6_name ));
        if ( right_loadx6==NULL ) {
            M3_WARN ( "M3LoadX6 component %s declared for M3BotShm but could not be linked\n",
                      right_loadx6_name.c_str() );
            //return false;
            }
        }

    if ( left_hand_name.size() !=0 ) {
		left_hand= dynamic_cast< M3Hand* >( factory->GetComponent ( left_hand_name )); //May be null if not on this robot model
        if ( left_hand==NULL ) {
            M3_WARN ( "M3Hand component %s declared for M3BotShm but could not be linked\n",
                      left_hand_name.c_str() );
            //return false;
            }
        }

    if ( left_gripper_name.size() !=0 ) {
		left_gripper= dynamic_cast< M3Joint* >( factory->GetComponent ( left_gripper_name )); //May be null if not on this robot model
        if ( left_gripper==NULL ) {
            M3_WARN ( "M3Gripper component %s declared for M3BotShm but could not be linked\n",
                      left_gripper_name.c_str() );
            }
        }

    if ( right_gripper_name.size() !=0 ) {
		right_gripper= dynamic_cast< M3Joint* >( factory->GetComponent ( right_gripper_name )); //May be null if not on this robot model
        if ( right_gripper==NULL ) {
            M3_WARN ( "M3Gripper component %s declared for M3BotShm but could not be linked\n",
                      right_gripper_name.c_str() );
            }
        }


    if ( left_loadx6_name.size() !=0 ) {
		left_loadx6= dynamic_cast< M3LoadX6* >( factory->GetComponent ( left_loadx6_name ));
        if ( left_loadx6==NULL ) {
            M3_WARN ( "M3LoadX6 component %s declared for M3BotShm but could not be linked\n",
                      left_loadx6_name.c_str() );
            //return false;
            }
        }

    if ( right_loadx6 || right_hand || bot || left_loadx6 || left_hand || left_gripper || right_gripper ) {
        return true;
        }
    else {
        M3_ERR ( "Could not link any components for M3BotShm shared memory.\n" );
        return false;
        }
    }

bool M3HumanoidShm::ReadConfig ( const char * filename ) {
    if ( !M3CompShm::ReadConfig ( filename ) )
        return false;

    //YAML::Node doc;
   // GetYamlDoc ( filename, doc );

    try {
        doc["humanoid_component"] >> bot_name;
        }
    catch ( ... ) {
        bot_name="";
        }

    try {
        doc["right_hand_component"] >> right_hand_name;
        }
    catch ( ... ) {
        right_hand_name="";
        }

    try {
        doc["right_loadx6_component"] >> right_loadx6_name;
        }
    catch ( ... ) {
        right_loadx6_name="";
        }

    try {
        doc["left_hand_component"] >> left_hand_name;
        }
    catch ( ... ) {
        left_hand_name="";
        }

    try {
        doc["left_gripper_component"] >> left_gripper_name;
        }
    catch ( ... ) {
        left_gripper_name="";
        }

    try {
        doc["right_gripper_component"] >> right_gripper_name;
        }
    catch ( ... ) {
        right_gripper_name="";
        }

    try {
        doc["left_loadx6_component"] >> left_loadx6_name;
        }
    catch ( ... ) {
        left_loadx6_name="";
        }

    try {
        doc["pwr_component"]>>pwr_name;
        }
    catch ( ... ) {
        pwr_name = "";
        }

    doc["timeout"] >> timeout;

    return true;
    }






} // namespace


// Copyright (c) 2019-2020, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Szymon Jarocki
//

#ifndef MASS_STATIC_CONTROL
#define MASS_STATIC_CONTROL

#include "rtt/RTT.hpp"
#include <rtt_rosparam/rosparam.h>

#include "kin_dyn_model/kin_model.h"

#include "ros/ros.h"

using namespace RTT;

template<unsigned int DOFS>
class MassStaticControl: public RTT::TaskContext
{
public:
    explicit MassStaticControl(const std::string &name);

    bool configureHook();
    bool startHook();
    void stopHook();
    void updateHook();

private:
    void calculateTorqueVector(uint8_t object_grasped, uint8_t *object_grasped_prev, int idx, KDL::Vector last_joint_mass_position_6);

    typedef Eigen::Matrix<double, DOFS, 1> VectorD;

    RTT::InputPort<VectorD> port_joint_position_;
    RTT::InputPort<VectorD> port_control_law_torque_command_;  
    RTT::OutputPort<VectorD> port_mass_static_torque_command_;    
    RTT::OutputPort<VectorD> port_joint_torque_command_;

    RTT::InputPort<uint8_t> port_object_grasped_right_command_;  
    RTT::InputPort<uint8_t> port_object_grasped_left_command_;
    RTT::InputPort<double> port_identified_weight_of_the_object_; 
    RTT::InputPort<KDL::Vector> port_identified_center_of_mass_location_right_; 
    RTT::InputPort<KDL::Vector> port_identified_center_of_mass_location_left_; 

    VectorD joint_position_;
    VectorD control_law_torque_command_;
    VectorD mass_static_torque_command_;
    VectorD joint_torque_command_;
    
    uint8_t object_grasped_right_, object_grasped_right_prev_;
    uint8_t object_grasped_left_, object_grasped_left_prev_;
    double identified_weight_of_the_object_;
    KDL::Vector position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_;

    std::string robot_description_;
    std::vector<std::string> articulated_joint_names_;

    boost::shared_ptr<KinematicModel> kin_model_;
    std::vector<KDL::Frame> links_fk_;
    std::vector<std::string> link_names_;

    std::vector<KDL::Vector> mass_position_, space_torque_, axis_local_, origin_local_, axis_0_;    
    KDL::Vector last_joint_mass_position_6_right_, last_joint_mass_position_6_left_;
    KDL::Vector last_joint_mass_position_0_, gravity_force_0_;

    // double mass_of_the_object_, gravitational_acceleration_;
    int rightLWR_joint0_idx_, leftLWR_joint0_idx_;
};

template<unsigned int DOFS>
MassStaticControl<DOFS>::MassStaticControl(const std::string &name)
    : TaskContext(name, PreOperational)
    , port_joint_position_("JointPosition_INPORT")
    , port_control_law_torque_command_("ControlLawTorqueCommand_INPORT")
    , port_object_grasped_right_command_("ObjectGraspedRightCommand_INPORT")
    , port_object_grasped_left_command_("ObjectGraspedLeftCommand_INPORT")
    , port_identified_weight_of_the_object_("IdentifiedWeightOfTheObject_INPORT") 
    , port_identified_center_of_mass_location_right_("IdentifiedCenterOfMassLocationRight_INPORT")        
    , port_identified_center_of_mass_location_left_("IdentifiedCenterOfMassLocationLeft_INPORT")               
    , port_joint_torque_command_("JointTorqueCommand_OUTPORT")
    , port_mass_static_torque_command_("MassStaticTorqueCommand_OUTPORT")    
{
    this->ports()->addPort(port_joint_position_);
    this->ports()->addPort(port_control_law_torque_command_);
    this->ports()->addPort(port_object_grasped_right_command_);   
    this->ports()->addPort(port_object_grasped_left_command_);
    this->ports()->addPort(port_identified_weight_of_the_object_);
    this->ports()->addPort(port_identified_center_of_mass_location_right_);
    this->ports()->addPort(port_identified_center_of_mass_location_left_);               
    this->ports()->addPort(port_joint_torque_command_);
    this->ports()->addPort(port_mass_static_torque_command_);

    this->addProperty("robot_description", robot_description_);
    this->addProperty("articulated_joint_names", articulated_joint_names_);
    this->addProperty("rightLWR_joint0_idx", rightLWR_joint0_idx_);
    this->addProperty("leftLWR_joint0_idx", leftLWR_joint0_idx_);    
    // this->addProperty("mass_of_the_object", mass_of_the_object_);
}

template<unsigned int DOFS>
bool MassStaticControl<DOFS>::configureHook()
{
    Logger::In in("MassStaticControl::configureHook");

    // Get the rosparam service requester
    boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");

      // Get the parameters
    if(!rosparam)
    {
        Logger::log() << Logger::Error << "Could not get ROS parameters from rtt_rosparam" << Logger::endl;
        return false;
    }

    // Get the ROS parameter "/robot_description"
    if (!rosparam->getAbsolute("robot_description"))
    {
        Logger::log() << Logger::Error << "could not read ROS parameter \'robot_description\'" << Logger::endl;
        return false;
    }

    if (articulated_joint_names_.size() != DOFS)
    {
        Logger::log() << Logger::Error << "ROS parameter \'articulated_joint_names\' has wrong size: " << articulated_joint_names_.size()
            << ", should be: " << DOFS << Logger::endl;
        return false;
    }

    kin_model_.reset( new KinematicModel(robot_description_, articulated_joint_names_) );

    link_names_.resize(DOFS);
    links_fk_.resize(DOFS);
    mass_position_.resize(7);
    space_torque_.resize(7);
    axis_local_.resize(7);
    axis_0_.resize(7);
    origin_local_.resize(7);

    mass_static_torque_command_.setZero();
    object_grasped_right_prev_ = false;
    object_grasped_left_prev_ = false;

    // gravitational_acceleration_ = 9.80665;

    // last_joint_mass_position_6_right_[0] = 0.27;
    // last_joint_mass_position_6_right_[1] = 0.0;
    // last_joint_mass_position_6_right_[2] = -0.078;

    // last_joint_mass_position_6_left_[0] = -0.27;
    // last_joint_mass_position_6_left_[1] = 0.0;
    // last_joint_mass_position_6_left_[2] = -0.078;

    SetToZero(last_joint_mass_position_6_right_);
    SetToZero(last_joint_mass_position_6_left_);
    SetToZero(gravity_force_0_);

    // gravity_force_0_[2] = -mass_of_the_object_ * gravitational_acceleration_;   

    for (int l_idx = 0; l_idx < DOFS; l_idx++)
    {
        kin_model_->getJointLinkName(l_idx, link_names_[l_idx]);
    }

    return true;
}

template<unsigned int DOFS>
bool MassStaticControl<DOFS>::startHook()
{
    //
    return true;
}

template<unsigned int DOFS>
void MassStaticControl<DOFS>::stopHook() 
{
    //  
}

template<unsigned int DOFS>
void MassStaticControl<DOFS>::updateHook() 
{
    if (port_joint_position_.read(joint_position_) != RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " could not read port \'" << port_joint_position_.getName() << "\'" << Logger::endl;
        return;
    }

    if (!joint_position_.allFinite()) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " joint_position_ contains NaN or inf" << Logger::endl;
        return;
    }

    if (port_control_law_torque_command_.read(control_law_torque_command_) != RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " could not read port \'" << port_control_law_torque_command_.getName() << "\'" << Logger::endl;
        return;
    }

    if (!control_law_torque_command_.allFinite()) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << getName() << " control_law_torque_command_ contains NaN or inf" << Logger::endl;
        return;
    }

    if (port_identified_weight_of_the_object_.read(identified_weight_of_the_object_) == RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        Logger::log() << "New data transfered to identified_weight_of_the_object_ variable" << Logger::endl;

        gravity_force_0_[2] = -identified_weight_of_the_object_;
    }

    if (port_identified_center_of_mass_location_right_.read(position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_) == RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        Logger::log() << "New data transfered to position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_ (right) variable" << Logger::endl;

        last_joint_mass_position_6_right_[0] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[0] + 0.115;
        last_joint_mass_position_6_right_[1] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[1];
        last_joint_mass_position_6_right_[2] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[2] - 0.078;        
    }

    if (port_identified_center_of_mass_location_left_.read(position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_) == RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        Logger::log() << "New data transfered to position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_ (left) variable" << Logger::endl;

        last_joint_mass_position_6_left_[0] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[0] - 0.115;
        last_joint_mass_position_6_left_[1] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[1];
        last_joint_mass_position_6_left_[2] = position_vector_from_ft_sensor_to_center_of_mass_location_in_wrist_frame_[2] - 0.078;                
    }

    if (port_object_grasped_right_command_.read(object_grasped_right_) == RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        Logger::log() << "New data transfered to object_grasped_right_ variable" << Logger::endl;
        //std::cout << "object_grasped_right_: " << object_grasped_right_  << std::endl;

        if (object_grasped_right_ == true)
        {
            object_grasped_right_prev_ = true;
        }
    }

    if (port_object_grasped_left_command_.read(object_grasped_left_) == RTT::NewData) 
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        Logger::log() << "New data transfered to object_grasped_left_ variable" << Logger::endl;
        //std::cout << "object_grasped_left_: " << object_grasped_left_  << std::endl;

        if (object_grasped_left_ == true)
        {
            object_grasped_left_prev_ = true;
        }
    }

    if (object_grasped_right_ == true || object_grasped_left_ == true)
    {
        kin_model_->calculateFkAll(joint_position_);

        for (int l_idx = 0; l_idx < DOFS; l_idx++)
        {
            links_fk_[l_idx] = kin_model_->getFrame(link_names_[l_idx]);
        }
    }

    calculateTorqueVector(object_grasped_right_, &object_grasped_right_prev_, rightLWR_joint0_idx_, last_joint_mass_position_6_right_);   
    calculateTorqueVector(object_grasped_left_, &object_grasped_left_prev_, leftLWR_joint0_idx_, last_joint_mass_position_6_left_);

    // for (int j = 0; j < DOFS; j++)
    // {
    //     std::cout << "mass_static_torque_command_" << j << "_: " << mass_static_torque_command_[j] << std::endl;
    //     std::cout << "control_law_torque_command_" << j << "_: " << control_law_torque_command_[j] << std::endl; 
    // }

    if (!mass_static_torque_command_.allFinite())
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << "mass_static_torque_command_ contains NaN or inf" << Logger::endl;
        return;
    }

    joint_torque_command_.noalias() = mass_static_torque_command_ + control_law_torque_command_;

    if (!joint_torque_command_.allFinite())
    {
        RTT::Logger::In in("MassStaticControl::updateHook");
        error();
        Logger::log() << Logger::Error << "joint_torque_command_ contains NaN or inf" << Logger::endl;
        return;
    }

    port_joint_torque_command_.write(joint_torque_command_);
    port_mass_static_torque_command_.write(mass_static_torque_command_);
}

template<unsigned int DOFS>
void MassStaticControl<DOFS>::calculateTorqueVector(uint8_t object_grasped, uint8_t* object_grasped_prev, int idx, KDL::Vector last_joint_mass_position_6)
{
    if (object_grasped == true)
    {
        last_joint_mass_position_0_ = links_fk_[idx+6].M * last_joint_mass_position_6;
        for (int j = 0; j < 7; j++)
        {
            mass_position_[j] = links_fk_[idx+6].p - links_fk_[idx+j].p + last_joint_mass_position_0_;
            // std::cout << mass_position_[j](0) << " " << mass_position_[j](1) << " " << mass_position_[j](2) << std::endl;
            space_torque_[j](0) = mass_position_[j](1)*gravity_force_0_[2]; //- mass_position_[j](2)*gravity_force_0_[1];
            space_torque_[j](1) = -mass_position_[j](0)*gravity_force_0_[2]; // + mass_position_[j](2)*gravity_force_0_[0]
            space_torque_[j](2) = 0.0; // mass_position_[j](0)*gravity_force_0_[1] - mass_position_[j](1)*gravity_force_0_[0];

            kin_model_->getJointAxisAndOrigin(idx+j, axis_local_[j], origin_local_[j]);
            axis_0_[j] = links_fk_[idx+j].M * axis_local_[j];
            mass_static_torque_command_[idx+j] = -(space_torque_[j](0)*axis_0_[j](0) + space_torque_[j](1)*axis_0_[j](1) + space_torque_[j](2)*axis_0_[j](2));
            // std::cout << "right_arm_" << j << "_mass_static_torque_command_: " << mass_static_torque_command_[j+1] << std::endl;
        }
    }
    else
    {
        if (*object_grasped_prev == true)
        {
            for (int j = 0; j < 7; j++)
            {
                mass_static_torque_command_[idx+j] = 0.0;
            }
            *object_grasped_prev = false;
        }
    }
} 

#endif


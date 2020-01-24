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

// #ifndef MASS_DYNAMIC_CONTROL
// #define MASS_DYNAMIC_CONTROL

// #include "controller_common/robot.h"
// #include <kdl/frames.hpp>

// using namespace RTT;

// template<unsigned int DOFS, unsigned int EFFECTORS>
// class MassDynamicControl: public RTT::TaskContext
// {
// public:
//     explicit MassDynamicControl(const std::string &name);

//     bool configureHook();
//     bool startHook();
//     void stopHook();
//     void updateHook();

// private:
//     typedef Eigen::Matrix<double, DOFS, 1> VectorD;
//     typedef Eigen::Matrix<double, DOFS, DOFS> MatrixDD;
//     typedef Eigen::Matrix<double, EFFECTORS * 6, EFFECTORS * 6> MatrixEE;
//     typedef Eigen::Matrix<double, 6, 6> Matrix66;    
//     typedef Eigen::Matrix<double, 3, 3> Matrix33;         
//     typedef controller_common::Robot<DOFS, EFFECTORS> Robot;
//     typedef Eigen::Matrix<double, EFFECTORS * 6, DOFS> JacobiMatrix;    
//     typedef Eigen::Matrix<double, 7, 1> Tool;

//     void transformVectorToSkewSymetricMatrix(KDL::Vector vector3_, Matrix33* skew_symetrix_matrix_);
//     void calculateObjectCartesianSpaceMassMatrix(uint8_t object_grasped, uint8_t* object_grasped_prev, Matrix33 mass_position_skew_symetrix_matrix_, Matrix66* object_cartesian_inertia_matrix_);

//     RTT::InputPort<VectorD> port_joint_position_;
//     RTT::InputPort<VectorD> port_joint_velocity_;
//     RTT::InputPort<VectorD> port_msc_joint_torque_command_;  
// //    RTT::OutputPort<VectorD> port_mass_dynamic_torque_command_;    
//     RTT::OutputPort<VectorD> port_joint_torque_command_;

//     RTT::InputPort<uint8_t> port_object_grasped_right_command_;  
//     RTT::InputPort<uint8_t> port_object_grasped_left_command_; 

//     VectorD joint_position_;
//     VectorD joint_velocity_;
//     VectorD msc_joint_torque_command_;
//     VectorD mass_dynamic_torque_command_;
//     VectorD joint_torque_command_;
    
//     uint8_t object_grasped_right_, object_grasped_right_prev_;
//     uint8_t object_grasped_left_, object_grasped_left_prev_;

//     VectorD joint_approximate_acceleration_, joint_velocity_prev_;
//     MatrixDD object_joint_inertia_matrix_;
//     MatrixEE object_cartesian_inertia_matrix_;
//     Matrix66 object_cartesian_inertia_matrix_right_, object_cartesian_inertia_matrix_left_;
//     Matrix33 mass_position_skew_symetrix_matrix_right_, mass_position_skew_symetrix_matrix_left_;

//     boost::shared_ptr<Robot> robot_;
//     boost::array<Tool, EFFECTORS> tools_;    
//     JacobiMatrix jacobian_;
  
//     KDL::Vector last_joint_mass_position_6_right_, last_joint_mass_position_6_left_;
//     double mass_of_the_object_;
// };

// template<unsigned int DOFS, unsigned int EFFECTORS>
// MassDynamicControl<DOFS, EFFECTORS>::MassDynamicControl(const std::string &name)
//     : TaskContext(name, PreOperational)
//     , port_joint_position_("JointPosition_INPORT")
//     , port_joint_velocity_("Jointvelocity_INPORT")
//     , port_msc_joint_torque_command_("MSCJointTorqueCommand_INPORT")
//     , port_object_grasped_right_command_("ObjectGraspedRightCommand_INPORT")
//     , port_object_grasped_left_command_("ObjectGraspedLeftCommand_INPORT")       
//     , port_joint_torque_command_("JointTorqueCommand_OUTPORT")
// //    , port_mass_dynamic_torque_command_("MassDynamicTorqueCommand_OUTPORT")
// {
//     this->ports()->addPort(port_joint_position_);
//     this->ports()->addPort(port_joint_velocity_);    
//     this->ports()->addPort(port_msc_joint_torque_command_);
//     this->ports()->addPort(port_object_grasped_right_command_);   
//     this->ports()->addPort(port_object_grasped_left_command_);   
//     this->ports()->addPort(port_joint_torque_command_);
// //    this->ports()->addPort(port_mass_dynamic_torque_command_);
 
//     this->addProperty("mass_of_the_object", mass_of_the_object_);
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// bool MassDynamicControl<DOFS, EFFECTORS>::configureHook()
// {
//     Logger::In in("MassDynamicControl::configureHook");

//     robot_ = this->getProvider<Robot>("robot");
//     if (!robot_)
//     {
//       Logger::log() << Logger::Error << "Unable to load RobotService" << Logger::endl;
//       return false;
//     }

//     if (robot_->dofs() != DOFS) 
//     {
//       Logger::log() << Logger::Error << "wrong number of DOFs: " << robot_->dofs() << ", expected " << DOFS << Logger::endl;
//       return false;
//     }

//     if (robot_->effectors() != EFFECTORS) 
//     {
//       Logger::log() << Logger::Error << "wrong number of effectors: " << robot_->effectors() << ", expected " << EFFECTORS << Logger::endl;
//       return false;
//     }

//     object_cartesian_inertia_matrix_.setZero();
//     joint_velocity_prev_.setZero();
//     object_grasped_right_prev_ = false;
//     object_grasped_left_prev_ = false;

//     last_joint_mass_position_6_right_[0] = 0.3;
//     last_joint_mass_position_6_right_[1] = 0.0;
//     last_joint_mass_position_6_right_[2] = -0.078;
//     transformVectorToSkewSymetricMatrix(last_joint_mass_position_6_right_, &mass_position_skew_symetrix_matrix_right_);

//     last_joint_mass_position_6_left_[0] = -0.3;
//     last_joint_mass_position_6_left_[1] = 0.0;
//     last_joint_mass_position_6_left_[2] = -0.078;
//     transformVectorToSkewSymetricMatrix(last_joint_mass_position_6_left_, &mass_position_skew_symetrix_matrix_left_);

//     for (int i = 0; i < EFFECTORS; i++)
//     {
//         tools_[i](0) = 0; //pos.position.x;
//         tools_[i](1) = 0; //pos.position.y;
//         tools_[i](2) = 0; //pos.position.z;

//         tools_[i](3) = 0; //pos.orientation.w;
//         tools_[i](4) = 0; //pos.orientation.x;
//         tools_[i](5) = 0; //pos.orientation.y;
//         tools_[i](6) = 1; //pos.orientation.z;       
//     }

//     return true;
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// bool MassDynamicControl<DOFS, EFFECTORS>::startHook()
// {
//     //
//     return true;
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// void MassDynamicControl<DOFS, EFFECTORS>::stopHook() 
// {
//     //  
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// void MassDynamicControl<DOFS, EFFECTORS>::updateHook() 
// {
//     if (port_joint_position_.read(joint_position_) != RTT::NewData) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " could not read port \'" << port_joint_position_.getName() << "\'" << Logger::endl;
//         return;
//     }

//     if (!joint_position_.allFinite()) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " joint_position_ contains NaN or inf" << Logger::endl;
//         return;
//     }

//     if (port_joint_velocity_.read(joint_velocity_) != RTT::NewData) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " could not read port \'" << port_joint_velocity_.getName() << "\'" << Logger::endl;
//         return;
//     }

//     if (!joint_velocity_.allFinite()) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " joint_velocity_ contains NaN or inf" << Logger::endl;
//         return;
//     }

//     if (port_msc_joint_torque_command_.read(msc_joint_torque_command_) != RTT::NewData) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " could not read port \'" << port_msc_joint_torque_command_.getName() << "\'" << Logger::endl;
//         return;
//     }

//     if (!msc_joint_torque_command_.allFinite()) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << getName() << " msc_joint_torque_command_ contains NaN or inf" << Logger::endl;
//         return;
//     }

//     if (port_object_grasped_right_command_.read(object_grasped_right_) == RTT::NewData) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         Logger::log() << "New data transfered to object_grasped_right_ variable" << Logger::endl;
//         //std::cout << "object_grasped_right_: " << object_grasped_right_  << std::endl;

//         if (object_grasped_right_ == true)
//         {
//             object_grasped_right_prev_ = true;
//         }
//     }

//     if (port_object_grasped_left_command_.read(object_grasped_left_) == RTT::NewData) 
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         Logger::log() << "New data transfered to object_grasped_left_ variable" << Logger::endl;
//         //std::cout << "object_grasped_left_: " << object_grasped_left_  << std::endl;

//         if (object_grasped_left_ == true)
//         {
//             object_grasped_left_prev_ = true;
//         }
//     }

//     joint_approximate_acceleration_ = (joint_velocity_ - joint_velocity_prev_) / 0.001;
//     joint_velocity_prev_ = joint_velocity_;

//     if (object_grasped_right_ == true || object_grasped_left_ == true)
//     {
//         robot_->jacobian(jacobian_, joint_position_, &tools_[0]);

//         if (!jacobian_.allFinite()) 
//         {
//             RTT::Logger::In in("CartesianImpedance::updateHook");
//             error();
//             Logger::log() << Logger::Error << " jacobian_ contains NaN or inf" << Logger::endl;
//             return;
//         }

//         calculateObjectCartesianSpaceMassMatrix(object_grasped_right_, &object_grasped_right_prev_, mass_position_skew_symetrix_matrix_right_, &object_cartesian_inertia_matrix_right_); 
//         calculateObjectCartesianSpaceMassMatrix(object_grasped_left_, &object_grasped_left_prev_, mass_position_skew_symetrix_matrix_left_, &object_cartesian_inertia_matrix_left_);
//         object_cartesian_inertia_matrix_.template block<6, 6>(0, 0) = object_cartesian_inertia_matrix_right_;
//         object_cartesian_inertia_matrix_.template block<6, 6>(6, 6) = object_cartesian_inertia_matrix_left_;

//         object_joint_inertia_matrix_ = jacobian_.transpose() * object_cartesian_inertia_matrix_ * jacobian_;
//         mass_dynamic_torque_command_ = object_joint_inertia_matrix_ * joint_approximate_acceleration_;        
//     }
//     else
//     {
//         mass_dynamic_torque_command_.setZero();
//     }

//     for (int j = 0; j < DOFS; j++)
//     {
//         std::cout << "mass_dynamic_torque_command_" << j << "_: " << mass_dynamic_torque_command_[j] << std::endl;
//         std::cout << "msc_joint_torque_command_" << j << "_: " << msc_joint_torque_command_[j] << std::endl; 
//     }

//     if (!mass_dynamic_torque_command_.allFinite())
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << " mass_dynamic_torque_command_ contains NaN or inf" << Logger::endl;
//         return;
//     }

// //    joint_torque_command_.noalias() = mass_dynamic_torque_command_ + msc_joint_torque_command_;
//     joint_torque_command_.noalias() = msc_joint_torque_command_;

//     if (!joint_torque_command_.allFinite())
//     {
//         RTT::Logger::In in("MassDynamicControl::updateHook");
//         error();
//         Logger::log() << Logger::Error << "joint_torque_command_ contains NaN or inf" << Logger::endl;
//         return;
//     }

//     port_joint_torque_command_.write(joint_torque_command_);
// //    port_mass_dynamic_torque_command_.write(mass_dynamic_torque_command_);
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// void MassDynamicControl<DOFS, EFFECTORS>::transformVectorToSkewSymetricMatrix(KDL::Vector vector3_, Matrix33* skew_symetrix_matrix_)
// {   
//     skew_symetrix_matrix_->setZero();
//     skew_symetrix_matrix_->operator()(0, 1) = -vector3_[2];
//     skew_symetrix_matrix_->operator()(0, 2) = vector3_[1];
//     skew_symetrix_matrix_->operator()(1, 0) = vector3_[2];
//     skew_symetrix_matrix_->operator()(1, 2) = -vector3_[0];
//     skew_symetrix_matrix_->operator()(2, 0) = -vector3_[1];
//     skew_symetrix_matrix_->operator()(2, 1) = vector3_[0];
// }

// template<unsigned int DOFS, unsigned int EFFECTORS>
// void MassDynamicControl<DOFS, EFFECTORS>::calculateObjectCartesianSpaceMassMatrix(uint8_t object_grasped, uint8_t* object_grasped_prev, Matrix33 mass_position_skew_symetrix_matrix_, Matrix66* object_cartesian_inertia_matrix_)
// {
//     if (object_grasped == true)
//     {
//         object_cartesian_inertia_matrix_->template block<3, 3>(0, 0) = mass_of_the_object_ * Matrix33::Identity();
//         object_cartesian_inertia_matrix_->template block<3, 3>(0, 3) = -mass_of_the_object_ * mass_position_skew_symetrix_matrix_;
//         object_cartesian_inertia_matrix_->template block<3, 3>(3, 0) = mass_of_the_object_ * mass_position_skew_symetrix_matrix_;
//         object_cartesian_inertia_matrix_->template block<3, 3>(3, 3) = -mass_of_the_object_ * mass_position_skew_symetrix_matrix_ * mass_position_skew_symetrix_matrix_;
//     }
//     else
//     {
//         object_cartesian_inertia_matrix_->setZero();

//         if (*object_grasped_prev == true)
//         {
//             *object_grasped_prev = false;
//         }
//     }    
// }

// #endif


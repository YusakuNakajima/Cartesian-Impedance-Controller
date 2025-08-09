#include "cartesian_impedance_controller/coulomb_viscous_friction_model.h"
#include <iostream>
#include <iomanip>

namespace cartesian_impedance_controller {

void CoulombViscousFrictionModel::initialize(const std::vector<std::string>& joint_names,
                                           const std::map<std::string, std::map<std::string, double>>& parameters) {
    coulomb_friction_params_.clear();
    viscous_friction_params_.clear();
    
    for (const auto& joint_name : joint_names) {
        auto joint_params = parameters.find(joint_name);
        if (joint_params != parameters.end()) {
            auto coulomb_it = joint_params->second.find("coulomb_friction");
            auto viscous_it = joint_params->second.find("viscous_friction");
            
            if (coulomb_it != joint_params->second.end()) {
                coulomb_friction_params_[joint_name] = coulomb_it->second;
            } else {
                coulomb_friction_params_[joint_name] = 0.0;
            }
            
            if (viscous_it != joint_params->second.end()) {
                viscous_friction_params_[joint_name] = viscous_it->second;
            } else {
                viscous_friction_params_[joint_name] = 0.0;
            }
        } else {
            coulomb_friction_params_[joint_name] = 0.0;
            viscous_friction_params_[joint_name] = 0.0;
        }
    }
}

Eigen::VectorXd CoulombViscousFrictionModel::computeFrictionTorque(const std::vector<std::string>& joint_names,
                                                                 const Eigen::VectorXd& joint_positions,
                                                                 const Eigen::VectorXd& joint_velocities,
                                                                 double dt) {
    Eigen::VectorXd friction_torque = Eigen::VectorXd::Zero(joint_velocities.size());
    
    for (size_t i = 0; i < joint_names.size() && i < joint_velocities.size(); ++i) {
        const std::string& joint_name = joint_names[i];
        
        auto coulomb_it = coulomb_friction_params_.find(joint_name);
        auto viscous_it = viscous_friction_params_.find(joint_name);
        
        if (coulomb_it != coulomb_friction_params_.end() && 
            viscous_it != viscous_friction_params_.end()) {
            
            double coulomb_friction = coulomb_it->second;
            double viscous_friction = viscous_it->second;
            double velocity = joint_velocities[i];
            
            // Apply coulomb + viscous friction model: tau_friction = coulomb * sign(velocity) + viscous * velocity
            double coulomb_term = (velocity > 0) ? coulomb_friction : -coulomb_friction;
            double viscous_term = viscous_friction * velocity;
            
            friction_torque[i] = coulomb_term + viscous_term;
        }
    }
    
    return friction_torque;
}

void CoulombViscousFrictionModel::updateParameters(const std::map<std::string, std::map<std::string, double>>& parameters) {
    for (const auto& joint_params : parameters) {
        const std::string& joint_name = joint_params.first;
        
        auto coulomb_it = joint_params.second.find("coulomb_friction");
        auto viscous_it = joint_params.second.find("viscous_friction");
        
        if (coulomb_it != joint_params.second.end()) {
            coulomb_friction_params_[joint_name] = coulomb_it->second;
        }
        
        if (viscous_it != joint_params.second.end()) {
            viscous_friction_params_[joint_name] = viscous_it->second;
        }
    }
}

} // namespace cartesian_impedance_controller
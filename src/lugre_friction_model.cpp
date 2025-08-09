#include "cartesian_impedance_controller/lugre_friction_model.h"
#include <iostream>
#include <iomanip>
#include <cmath>

namespace cartesian_impedance_controller {

void LuGreFrictionModel::initialize(const std::vector<std::string>& joint_names,
                                   const std::map<std::string, std::map<std::string, double>>& parameters) {
    sigma0_params_.clear();
    sigma1_params_.clear();
    sigma2_params_.clear();
    coulomb_friction_params_.clear();
    static_friction_params_.clear();
    stribeck_velocity_params_.clear();
    bristle_state_.clear();
    
    for (const auto& joint_name : joint_names) {
        auto joint_params = parameters.find(joint_name);
        if (joint_params != parameters.end()) {
            // Load LuGre parameters with defaults
            auto sigma0_it = joint_params->second.find("sigma0");
            sigma0_params_[joint_name] = (sigma0_it != joint_params->second.end()) ? 
                                        sigma0_it->second : 1000.0;
            
            auto sigma1_it = joint_params->second.find("sigma1");
            sigma1_params_[joint_name] = (sigma1_it != joint_params->second.end()) ? 
                                        sigma1_it->second : 10.0;
            
            auto sigma2_it = joint_params->second.find("sigma2");
            sigma2_params_[joint_name] = (sigma2_it != joint_params->second.end()) ? 
                                        sigma2_it->second : 0.4;
            
            auto coulomb_it = joint_params->second.find("coulomb_friction");
            coulomb_friction_params_[joint_name] = (coulomb_it != joint_params->second.end()) ? 
                                                  coulomb_it->second : 1.0;
            
            auto static_it = joint_params->second.find("static_friction");
            static_friction_params_[joint_name] = (static_it != joint_params->second.end()) ? 
                                                 static_it->second : 1.5;
            
            auto stribeck_it = joint_params->second.find("stribeck_velocity");
            stribeck_velocity_params_[joint_name] = (stribeck_it != joint_params->second.end()) ? 
                                                   stribeck_it->second : 0.01;
        } else {
            // Default parameters if joint not found
            sigma0_params_[joint_name] = 1000.0;
            sigma1_params_[joint_name] = 10.0;
            sigma2_params_[joint_name] = 0.4;
            coulomb_friction_params_[joint_name] = 1.0;
            static_friction_params_[joint_name] = 1.5;
            stribeck_velocity_params_[joint_name] = 0.01;
        }
        
        // Initialize bristle deflection state to zero
        bristle_state_[joint_name] = 0.0;
    }
}

Eigen::VectorXd LuGreFrictionModel::computeFrictionTorque(const std::vector<std::string>& joint_names,
                                                         const Eigen::VectorXd& joint_positions,
                                                         const Eigen::VectorXd& joint_velocities,
                                                         double dt) {
    Eigen::VectorXd friction_torque = Eigen::VectorXd::Zero(joint_velocities.size());
    
    for (size_t i = 0; i < joint_names.size() && i < joint_velocities.size(); ++i) {
        const std::string& joint_name = joint_names[i];
        
        auto sigma0_it = sigma0_params_.find(joint_name);
        auto sigma1_it = sigma1_params_.find(joint_name);
        auto sigma2_it = sigma2_params_.find(joint_name);
        auto coulomb_it = coulomb_friction_params_.find(joint_name);
        auto static_it = static_friction_params_.find(joint_name);
        auto stribeck_it = stribeck_velocity_params_.find(joint_name);
        auto bristle_it = bristle_state_.find(joint_name);
        
        if (sigma0_it != sigma0_params_.end() && 
            sigma1_it != sigma1_params_.end() && 
            sigma2_it != sigma2_params_.end() &&
            coulomb_it != coulomb_friction_params_.end() &&
            static_it != static_friction_params_.end() &&
            stribeck_it != stribeck_velocity_params_.end() &&
            bristle_it != bristle_state_.end()) {
            
            double sigma0 = sigma0_it->second;
            double sigma1 = sigma1_it->second;
            double sigma2 = sigma2_it->second;
            double coulomb = coulomb_it->second;
            double static_friction = static_it->second;
            double vs = stribeck_it->second;
            double velocity = joint_velocities[i];
            double& z = bristle_it->second; // Reference to bristle state
            
            // Compute g(velocity) function
            double g_velocity = computeGFunction(velocity, coulomb, static_friction, vs);
            
            // Update bristle deflection state: dz/dt = velocity - sigma0 * |velocity| * z / g(velocity)
            double dz_dt = 0.0;
            if (std::abs(g_velocity) > 1e-8) { // Avoid division by zero
                dz_dt = velocity - (sigma0 * std::abs(velocity) * z) / g_velocity;
            } else {
                dz_dt = velocity; // Fallback if g_velocity is too small
            }
            
            // Integrate bristle state using Euler method
            z += dz_dt * dt;
            
            // Compute friction torque: tau_friction = sigma0 * z + sigma1 * dz + sigma2 * velocity
            friction_torque[i] = sigma0 * z + sigma1 * dz_dt + sigma2 * velocity;
        }
    }
    
    return friction_torque;
}

void LuGreFrictionModel::updateParameters(const std::map<std::string, std::map<std::string, double>>& parameters) {
    for (const auto& joint_params : parameters) {
        const std::string& joint_name = joint_params.first;
        
        // Update parameters if they exist
        auto sigma0_it = joint_params.second.find("sigma0");
        if (sigma0_it != joint_params.second.end()) {
            sigma0_params_[joint_name] = sigma0_it->second;
        }
        
        auto sigma1_it = joint_params.second.find("sigma1");
        if (sigma1_it != joint_params.second.end()) {
            sigma1_params_[joint_name] = sigma1_it->second;
        }
        
        auto sigma2_it = joint_params.second.find("sigma2");
        if (sigma2_it != joint_params.second.end()) {
            sigma2_params_[joint_name] = sigma2_it->second;
        }
        
        auto coulomb_it = joint_params.second.find("coulomb_friction");
        if (coulomb_it != joint_params.second.end()) {
            coulomb_friction_params_[joint_name] = coulomb_it->second;
        }
        
        auto static_it = joint_params.second.find("static_friction");
        if (static_it != joint_params.second.end()) {
            static_friction_params_[joint_name] = static_it->second;
        }
        
        auto stribeck_it = joint_params.second.find("stribeck_velocity");
        if (stribeck_it != joint_params.second.end()) {
            stribeck_velocity_params_[joint_name] = stribeck_it->second;
        }
    }
}

double LuGreFrictionModel::computeGFunction(double velocity, double coulomb, double static_friction, double vs) const {
    // g(velocity) = coulomb + (static - coulomb) * exp(-(velocity/vs)^2)
    double velocity_ratio = velocity / vs;
    double exp_term = std::exp(-(velocity_ratio * velocity_ratio));
    return coulomb + (static_friction - coulomb) * exp_term;
}

} // namespace cartesian_impedance_controller
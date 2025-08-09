#pragma once

#include "friction_model.h"
#include <map>

namespace cartesian_impedance_controller {

/**
 * @brief LuGre friction model implementation
 * 
 * This model implements the LuGre dynamic friction model:
 * tau_friction = sigma0 * z + sigma1 * dz + sigma2 * velocity
 * dz/dt = velocity - sigma0 * |velocity| * z / g(velocity)
 * g(velocity) = coulomb + (static - coulomb) * exp(-(velocity/vs)^2)
 * 
 * Parameters per joint:
 * - sigma0: Stiffness coefficient [Nm/rad]
 * - sigma1: Damping coefficient [Nm*s/rad]  
 * - sigma2: Viscous coefficient [Nm*s/rad]
 * - coulomb_friction: Coulomb friction [Nm]
 * - static_friction: Static friction [Nm]
 * - stribeck_velocity: Stribeck velocity [rad/s]
 */
class LuGreFrictionModel : public FrictionModel {
public:
    LuGreFrictionModel() = default;
    ~LuGreFrictionModel() override = default;

    void initialize(const std::vector<std::string>& joint_names,
                   const std::map<std::string, std::map<std::string, double>>& parameters) override;

    Eigen::VectorXd computeFrictionTorque(const std::vector<std::string>& joint_names,
                                         const Eigen::VectorXd& joint_positions,
                                         const Eigen::VectorXd& joint_velocities,
                                         double dt) override;

    void updateParameters(const std::map<std::string, std::map<std::string, double>>& parameters) override;

    std::string getModelType() const override { return "LuGre"; }

private:
    // LuGre model parameters per joint
    std::map<std::string, double> sigma0_params_;
    std::map<std::string, double> sigma1_params_;
    std::map<std::string, double> sigma2_params_;
    std::map<std::string, double> coulomb_friction_params_;
    std::map<std::string, double> static_friction_params_;
    std::map<std::string, double> stribeck_velocity_params_;
    
    // Internal state (bristle deflection) per joint
    std::map<std::string, double> bristle_state_;
    
    /**
     * @brief Compute g(velocity) function for LuGre model
     */
    double computeGFunction(double velocity, double coulomb, double static_friction, double vs) const;
};

} // namespace cartesian_impedance_controller
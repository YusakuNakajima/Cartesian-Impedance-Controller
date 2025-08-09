#pragma once

#include "friction_model.h"
#include <map>

namespace cartesian_impedance_controller {

/**
 * @brief Coulomb + Viscous friction model implementation
 * 
 * This model implements the classic friction model:
 * tau_friction = coulomb * sign(velocity) + viscous * velocity
 * 
 * Parameters per joint:
 * - coulomb_friction: Coulomb friction magnitude [Nm]
 * - viscous_friction: Viscous friction coefficient [Nm*s/rad]
 */
class CoulombViscousFrictionModel : public FrictionModel {
public:
    CoulombViscousFrictionModel() = default;
    ~CoulombViscousFrictionModel() override = default;

    void initialize(const std::vector<std::string>& joint_names,
                   const std::map<std::string, std::map<std::string, double>>& parameters) override;

    Eigen::VectorXd computeFrictionTorque(const std::vector<std::string>& joint_names,
                                         const Eigen::VectorXd& joint_positions,
                                         const Eigen::VectorXd& joint_velocities,
                                         double dt) override;

    void updateParameters(const std::map<std::string, std::map<std::string, double>>& parameters) override;

    std::string getModelType() const override { return "CoulombViscous"; }

private:
    std::map<std::string, double> coulomb_friction_params_;
    std::map<std::string, double> viscous_friction_params_;
};

} // namespace cartesian_impedance_controller
#pragma once

#include <string>
#include <map>
#include <vector>
#include <Eigen/Dense>

namespace cartesian_impedance_controller {

/**
 * @brief Abstract base class for friction models
 * 
 * This class provides an interface for different friction models that can be used
 * in the Cartesian impedance controller. Derived classes should implement specific
 * friction models such as Coulomb+Viscous, LuGre, etc.
 */
class FrictionModel {
public:
    virtual ~FrictionModel() = default;

    /**
     * @brief Initialize the friction model with joint names and parameters
     * @param joint_names List of joint names
     * @param parameters Model-specific parameters
     */
    virtual void initialize(const std::vector<std::string>& joint_names,
                          const std::map<std::string, std::map<std::string, double>>& parameters) = 0;

    /**
     * @brief Compute friction compensation torques
     * @param joint_names List of joint names
     * @param joint_positions Current joint positions [rad]
     * @param joint_velocities Current joint velocities [rad/s]
     * @param dt Time step [s]
     * @return Friction compensation torques [Nm]
     */
    virtual Eigen::VectorXd computeFrictionTorque(const std::vector<std::string>& joint_names,
                                                 const Eigen::VectorXd& joint_positions,
                                                 const Eigen::VectorXd& joint_velocities,
                                                 double dt) = 0;

    /**
     * @brief Update friction model parameters at runtime
     * @param parameters Updated parameters
     */
    virtual void updateParameters(const std::map<std::string, std::map<std::string, double>>& parameters) = 0;

    /**
     * @brief Get model type name
     * @return String identifier for the friction model
     */
    virtual std::string getModelType() const = 0;
};

} // namespace cartesian_impedance_controller
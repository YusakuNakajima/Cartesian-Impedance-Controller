# Friction Compensation Usage Example

## Overview
The enhanced Cartesian Impedance Controller now supports sophisticated friction compensation with per-joint coulomb and viscous friction parameters.

## Usage Options

### 1. Using YAML File (Recommended)
Launch with friction parameters file:
```bash
roslaunch cartesian_impedance_controller examples.launch friction_params_file:="$(find cartesian_impedance_controller)/res/config/example_friction_params.yaml"
```

### 2. Using Dynamic Reconfigure
Launch controller normally and use rqt_reconfigure:
```bash
roslaunch cartesian_impedance_controller examples.launch
rqt_reconfigure
```

Then navigate to `/CartesianImpedance_trajectory_controller/friction_compensation_reconfigure` and:
- Set `friction_enable` to `true`
- Adjust individual joint friction parameters (joint1_coulomb_friction, etc.)
- Click `update_friction` to apply changes

### 3. Reloading Friction Parameters
In dynamic reconfigure:
- Set `friction_params_file` to the YAML file path
- Set `reload_friction_file` to `true`
- The controller will reload parameters from the specified file

## Friction Model
The system uses a comprehensive friction model:
```
tau_friction = coulomb_friction * sign(velocity) + viscous_friction * velocity
```

Where:
- `coulomb_friction`: Static friction coefficient (N·m)
- `viscous_friction`: Velocity-dependent friction coefficient (N·m·s/rad)

## Configuration Details

### YAML File Format
```yaml
joint1:
  coulomb_friction: 2.465
  viscous_friction: 0.339
joint2:
  coulomb_friction: 3.337
  viscous_friction: 0.083
# ... (for each joint)
```

### Key Features
- **Automatic fallback**: If YAML file not specified, friction parameters default to zero
- **Joint name matching**: Friction parameters must match controller joint names exactly
- **Real-time adjustment**: Parameters can be modified via dynamic reconfigure without restarting
- **Mixed mode support**: Can use both YAML initialization and dynamic reconfigure adjustment

## Dynamic Reconfigure Interface
- All per-joint friction parameters start at **0.0** in dynamic reconfigure
- When YAML file is loaded, values are automatically reflected in dynamic reconfigure
- Users can then fine-tune parameters in real-time through rqt_reconfigure

## Parameter Flow
1. **Startup**: All dynamic reconfigure parameters initialize to 0.0
2. **YAML Loading**: If friction_params_file is specified, values are loaded and updated in dynamic reconfigure
3. **Real-time Adjustment**: Users can modify parameters through rqt_reconfigure interface
4. **File Reload**: Use reload_friction_file option to re-read YAML and update interface

## Safety Notes
- Friction compensation only activates when position error exceeds the threshold
- Parameters are bounded within safe ranges in dynamic reconfigure
- Controller maintains backward compatibility with legacy static friction compensation

## Example Values
The example_friction_params.yaml contains real friction measurements that can be used as a starting point for tuning your specific robot setup.
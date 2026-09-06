#ifndef OCTO_PLANNER_GRADIENT_FOOTPRINT_MODEL_H_
#define OCTO_PLANNER_GRADIENT_FOOTPRINT_MODEL_H_

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

namespace octo_planner
{

/**
 * @brief Control disk definition in robot body coordinate frame.
 * Used to discretize any rigid footprint into control disks for continuous potential field evaluation.
 */
struct FootprintControlDisk
{
  double dx = 0.0;      ///< Longitudinal offset along robot heading (x_body, forward positive, in meters)
  double dy = 0.0;      ///< Lateral offset perpendicular to robot heading (y_body, left positive, in meters)
  double radius = 0.0;  ///< Equivalent collision/clearance radius of this disk (in meters)
  double weight = 1.0;  ///< Importance weight for gradient and cost scaling
};

/**
 * @class BaseGradientFootprintModel
 * @brief Abstract interface adhering to the Open-Closed Principle (OCP).
 * Enables plugging in arbitrary footprint shapes (TwoSpheres, FourFeet, Polygon, etc.)
 * without modifying cost calculation or optimizer edges.
 */
class BaseGradientFootprintModel
{
public:
  virtual ~BaseGradientFootprintModel() = default;

  /**
   * @brief Get the list of control disks in the robot body frame.
   */
  virtual const std::vector<FootprintControlDisk>& getDisks() const = 0;

  /**
   * @brief Get the inscribed radius of the footprint (for coarse bounding checks).
   */
  virtual double getInscribedRadius() const = 0;

  /**
   * @brief Get the circumscribed radius of the footprint (for bounding checks).
   */
  virtual double getCircumscribedRadius() const = 0;
};

using GradientFootprintModelPtr = std::shared_ptr<BaseGradientFootprintModel>;
using GradientFootprintModelConstPtr = std::shared_ptr<const BaseGradientFootprintModel>;

/**
 * @class TwoSpheresGradientFootprint
 * @brief Two-sphere / two-circle footprint model for quadruped robot bodies.
 * Front and rear spheres capture body pitch/yaw envelope and enforce rotational torque arms near cliffs.
 */
class TwoSpheresGradientFootprint : public BaseGradientFootprintModel
{
public:
  TwoSpheresGradientFootprint(double front_offset, double front_radius,
                              double rear_offset, double rear_radius,
                              double front_weight = 1.0, double rear_weight = 1.0)
    : front_offset_(front_offset), front_radius_(front_radius),
      rear_offset_(rear_offset), rear_radius_(rear_radius)
  {
    disks_.clear();
    disks_.push_back(FootprintControlDisk{front_offset, 0.0, front_radius, front_weight});
    disks_.push_back(FootprintControlDisk{-rear_offset, 0.0, rear_radius, rear_weight});
  }

  const std::vector<FootprintControlDisk>& getDisks() const override
  {
    return disks_;
  }

  double getInscribedRadius() const override
  {
    return std::min(front_radius_, rear_radius_);
  }

  double getCircumscribedRadius() const override
  {
    return std::max(front_offset_ + front_radius_, rear_offset_ + rear_radius_);
  }

private:
  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;
  std::vector<FootprintControlDisk> disks_;
};

/**
 * @class PointGradientFootprint
 * @brief Single-point / single-disk footprint model for circular or point robots.
 */
class PointGradientFootprint : public BaseGradientFootprintModel
{
public:
  explicit PointGradientFootprint(double radius = 0.20, double weight = 1.0)
    : radius_(radius)
  {
    disks_.clear();
    disks_.push_back(FootprintControlDisk{0.0, 0.0, radius, weight});
  }

  const std::vector<FootprintControlDisk>& getDisks() const override
  {
    return disks_;
  }

  double getInscribedRadius() const override
  {
    return radius_;
  }

  double getCircumscribedRadius() const override
  {
    return radius_;
  }

private:
  double radius_;
  std::vector<FootprintControlDisk> disks_;
};

} // namespace octo_planner

#endif // OCTO_PLANNER_GRADIENT_FOOTPRINT_MODEL_H_

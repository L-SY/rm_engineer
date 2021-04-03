#pragma once

#include "engineer_middleware/arm_motion/arm_motion_base.h"
#include "engineer_middleware/base_motion/base_motion_base.h"

// STL
#include <string>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <string>

namespace engineer_middleware {

class LegMotionBase;
class BaseMotionBase;

class Step {
 public:
  Step();
  ~Step() = default;
  Step(const Step &other);
  Step &operator=(const Step &other);

  std::unique_ptr<Step> clone() const;
  /*!
   * Add swing data for a arm.
   * @param data the step data.
   */
  void addArmMotion(const LegMotionBase &legMotion);

  /*!
   * Add base shift data for a state.
   * @param state the corresponding state of the base shift data.
   * @param data the base shift data.
   */
  void addBaseMotion(const BaseMotionBase &baseMotion);

  bool needsComputation() const;
  bool compute();

  bool update();
  bool isUpdated() const;
  void reset();

  /*!
   * Advance in time
   * @param dt the time step to advance [s].
   * @return true if step is active, false if finished.
   */
  bool advance(double dt);

  bool hasArmMotion() const { return (bool) (arm_motion_); }
  const BaseMotionBase &getBaseMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getBaseMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No base motion in this step!");
    return *base_motion_;
  };

  bool hasBaseMotion() const { return (bool) (base_motion_); };
  const BaseMotionBase &getArmMotion() const {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };
  BaseMotionBase &getArmMotion() {
    if (!hasBaseMotion()) throw std::out_of_range("No arm motion in this step!");
    return *base_motion_;
  };

  /*!
   * Return the current time of the step, starting at 0.0 for each step.
   * @return the current time.
   */
  double getTime() const;
  double getTotalDuration() const;
  double getTotalPhase() const;
  double getArmMotionDuration() const;
  double getBaseMotionDuration() const;
  double getBaseMotionPhase() const;

  bool isApproachingEnd(double tolerance) const;

  const std::string &getId() const;
  void setId(const std::string &id);

  friend std::ostream &operator<<(std::ostream &out, const Step &step);

  friend class StepCompleter;

 protected:
  std::unique_ptr<ArmMotionBase> arm_motion_;
  std::unique_ptr<BaseMotionBase> base_motion_;

 private:
  //! Current time, starts at 0.0 at each step.
  double time_;
  double total_duration_;
  bool is_updated_;
  bool is_computed_;
  std::string id_;
};

} /* namespace */

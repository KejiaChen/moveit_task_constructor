/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Hamburg University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael 'v4hn' Goerner
   Desc:   Define implementations for general CostTerm's to use with Stage::setCostTerm()
*/

#pragma once

#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/utils.h>
#include <moveit_msgs/msg/robot_state.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace moveit {
namespace task_constructor {

/** basic interface to compute costs for solutions
 *
 * If your cost term will only work on SubTrajectory solution objects,
 * inherit from TrajectoryCostTerm instead.
 */

MOVEIT_CLASS_FORWARD(CostTerm);
class CostTerm
{
public:
	CostTerm() = default;
	CostTerm(std::nullptr_t) : CostTerm{} {}
	virtual ~CostTerm() = default;

	virtual double operator()(const SubTrajectory& s, std::string& comment) const;
	virtual double operator()(const SolutionSequence& s, std::string& comment) const;
	virtual double operator()(const WrappedSolution& s, std::string& comment) const;
};

/** base class for cost terms that only work on SubTrajectory solutions
 *
 */
class TrajectoryCostTerm : public CostTerm
{
public:
	enum class Mode
	{
		AUTO /* TRAJECTORY, or START_INTERFACE if no trajectory is given */,
		START_INTERFACE,
		END_INTERFACE,
		TRAJECTORY
	};

	double operator()(const SolutionSequence& s, std::string& comment) const override;
	double operator()(const WrappedSolution& s, std::string& comment) const override;
};

class LambdaCostTerm : public TrajectoryCostTerm
{
public:
	using SubTrajectorySignature = std::function<double(const SubTrajectory&, std::string&)>;
	using SubTrajectoryShortSignature = std::function<double(const SubTrajectory&)>;

	// accept lambdas according to either signature above
	template <typename Term, typename Signature = decltype(signatureMatcher(std::declval<Term>()))>
	LambdaCostTerm(const Term& t) : LambdaCostTerm{ Signature{ t } } {}

	LambdaCostTerm(const SubTrajectorySignature& term);
	LambdaCostTerm(const SubTrajectoryShortSignature& term);

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

protected:
	SubTrajectorySignature term_;

private:
	template <typename T>
	static auto signatureMatcher(const T& t) -> decltype(t(SubTrajectory{}), SubTrajectoryShortSignature{});
	template <typename T>
	static auto signatureMatcher(const T& t) -> decltype(t(SubTrajectory{}, std::string{}), SubTrajectorySignature{});
};

namespace cost {

/// add a constant cost to each solution
class Constant : public CostTerm
{
public:
	Constant(double c) : cost{ c } {};

	double operator()(const SubTrajectory& s, std::string& comment) const override;
	double operator()(const SolutionSequence& s, std::string& comment) const override;
	double operator()(const WrappedSolution& s, std::string& comment) const override;

	double cost;
};

/// trajectory length with optional weighting for different joints
class PathLength : public TrajectoryCostTerm
{
public:
	/// By default, all joints are considered with same weight of 1.0
	PathLength() = default;
	/// Limit measurements to given joint names
	PathLength(std::vector<std::string> joints);
	/// Limit measurements to given joints and use given weighting
	PathLength(std::map<std::string, double> j) : joints(std::move(j)) {}

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

	std::map<std::string, double> joints;  //< joint weights
};

/// (weighted) joint-space distance to reference pose
class DistanceToReference : public TrajectoryCostTerm
{
public:
	DistanceToReference(const moveit_msgs::msg::RobotState& ref, Mode m = Mode::AUTO,
	                    std::map<std::string, double> w = std::map<std::string, double>());
	DistanceToReference(const std::map<std::string, double>& ref, Mode m = Mode::AUTO,
	                    std::map<std::string, double> w = std::map<std::string, double>());

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

	moveit_msgs::msg::RobotState reference;
	std::map<std::string, double> weights;
	Mode mode;
};

/// execution duration of the whole trajectory
class TrajectoryDuration : public TrajectoryCostTerm
{
public:
	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

/** length of Cartesian trajection of ONE link */
class LinkMotion : public TrajectoryCostTerm
{
public:
	LinkMotion(std::string link_name);

	std::string link_name;

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

/** Sum of length of Cartesian trajection of some links */
class LinkMotionSum : public TrajectoryCostTerm
{
public:
	LinkMotionSum(std::vector<std::string> links, std::vector<Eigen::Isometry3d> offsets);

	std::vector<std::string> link_names;
	std::vector<Eigen::Isometry3d> offsets;  //< offsets to apply to each link's position

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

/* Manipulability in a specific direction */
class DirectionalManipulability: public TrajectoryCostTerm
{
public:
  enum class Space { TRANSLATION, ROTATION };
  /// direction_vec is expressed in direction_frame. If direction_frame=="world" it's used as-is.
  DirectionalManipulability(
	std::map<std::string, std::string> group_ee,
	geometry_msgs::msg::Vector3Stamped direction_vec,
	Space space = Space::TRANSLATION,                              // TRANSLATION or ROTATION
	std::map<std::string,double> group_weights = std::map<std::string,double>(), // group-name -> weight (optional)
	double epsilon=1e-6,
	Mode mode=Mode::AUTO);

  using TrajectoryCostTerm::operator();
  double operator()(const SubTrajectory& s, std::string& comment) const override;

private:																		
	double evalState(const moveit::core::RobotState& state_in, const Eigen::Vector3d& u_world) const;

  std::map<std::string, std::string> group_ee_; // group-name -> end-effector link name
  geometry_msgs::msg::Vector3Stamped direction_vec_; // direction in which manipulability is evaluated
  std::string dir_frame_; // frame in which direction_vec is expressed
  Eigen::Vector3d dir_local_;
  Space space_;
  std::map<std::string,double> group_weights_; // group-name -> weight
  double eps_;
  Mode  mode_;

};

// ---------- Helpers ----------
inline double sigmaMinFullJ(const moveit::core::RobotState& rs,
                            const moveit::core::JointModelGroup* jmg,
                            const std::string& tip_link)
{
  Eigen::MatrixXd J;
  rs.getJacobian(jmg, rs.getLinkModel(tip_link), Eigen::Vector3d::Zero(), J);  // 6×N
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.singularValues().minCoeff();
}

// Smooth quadratic penalty that turns on below warn
//  s >= warn         -> 0
//  crit < s < warn   -> ((warn - s)/warn)^2
//  s <= crit         -> (hard_gate ? inf : big penalty)
inline double sigmaPenalty(double s, double warn, double crit, bool hard_gate, double big = 1e6)
{
  if (s >= warn) return 0.0;
  if (s <= crit) return hard_gate ? std::numeric_limits<double>::infinity() : big;
  const double d = (warn - s) / std::max(1e-12, warn);
  return d * d;
}

/* ManipulabilitySoftPenalty */
class ManipulabilitySoftPenalty : public moveit::task_constructor::CostTerm
{
public:
  // group_ee: map "group_name" -> "ee_link_name" (evaluate worst arm)
  explicit ManipulabilitySoftPenalty(std::map<std::string, std::string> group_ee,
                                     double sigma_warn = 3e-3,
                                     double sigma_crit = 1e-3,
                                     bool hard_gate = true,
                                     double weight = 1.0)
  : group_ee_(std::move(group_ee)),
    sigma_warn_(sigma_warn), sigma_crit_(sigma_crit),
    hard_gate_(hard_gate), weight_(weight) {}

  double operator()(const moveit::task_constructor::SubTrajectory& s,
                    std::string& comment) const override;

  void setThresholds(double warn, double crit) { sigma_warn_ = warn; sigma_crit_ = crit; }
  void setHardGate(bool on) { hard_gate_ = on; }
  void setWeight(double w) { weight_ = w; }

private:
  std::map<std::string, std::string> group_ee_;
  double sigma_warn_, sigma_crit_;
  bool hard_gate_;
  double weight_;
};

/* WeightedSumCost (combines any cost terms) */
class WeightedSumCost : public moveit::task_constructor::CostTerm
{
public:
  // Add any number of (cost_term, weight) pairs
  using TermW = std::pair<std::shared_ptr<moveit::task_constructor::CostTerm>, double>;

  WeightedSumCost() = default;
  explicit WeightedSumCost(std::initializer_list<TermW> list) : terms_(list) {}

  void add(const std::shared_ptr<moveit::task_constructor::CostTerm>& term, double weight = 1.0)
  { terms_.emplace_back(term, weight); }

  double operator()(const moveit::task_constructor::SubTrajectory& s,
                    std::string& comment) const override;

private:
  std::vector<TermW> terms_;
};

/** inverse distance to collision
 *
 * \arg with_world check distances to world objects or look at self-collisions
 * \arg cumulative if true, compute clearance as aggregated distance of all bodies
 * \arg group_property the name of the property which defines the group to look at
 * \arg interface compute distances using START or END interface of solution *only*, instead of averaging over
 * trajectory
 * */
class Clearance : public TrajectoryCostTerm
{
public:
	Clearance(bool with_world = true, bool cumulative = false, std::string group_property = "group",
	          Mode mode = Mode::AUTO);
	bool with_world;
	bool cumulative;
	std::string group_property;

	Mode mode;

	std::function<double(double)> distance_to_cost;

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

}  // namespace cost
}  // namespace task_constructor
}  // namespace moveit

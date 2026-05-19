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
 *   * Neither the name of the copyright holders nor the names of its
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

/* Authors: Michael Goerner */

#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/fmt_p.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>

#include <utility>

namespace moveit {
namespace task_constructor {

double CostTerm::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	return s.cost();
}

double CostTerm::operator()(const SolutionSequence& s, std::string& /*comment*/) const {
	return s.cost();
}

double CostTerm::operator()(const WrappedSolution& s, std::string& /*comment*/) const {
	return s.cost();
}

double TrajectoryCostTerm::operator()(const SolutionSequence& s, std::string& comment) const {
	double cost{ 0.0 };
	std::string subcomment;
	for (auto& solution : s.solutions()) {
		cost += solution->computeCost((*this), subcomment);
		if (!subcomment.empty()) {
			if (!comment.empty())
				comment.append(", ");
			comment.append(subcomment);
			subcomment.clear();
		}
	}

	return cost;
}

double TrajectoryCostTerm::operator()(const WrappedSolution& s, std::string& comment) const {
	return s.wrapped()->computeCost(*this, comment);
}

LambdaCostTerm::LambdaCostTerm(const SubTrajectorySignature& term)
  : term_{ [term](const SolutionBase& s, std::string& c) { return term(static_cast<const SubTrajectory&>(s), c); } } {}

LambdaCostTerm::LambdaCostTerm(const SubTrajectoryShortSignature& term)
  : term_{ [term](const SolutionBase& s, std::string& /*c*/) { return term(static_cast<const SubTrajectory&>(s)); } } {}

double LambdaCostTerm::operator()(const SubTrajectory& s, std::string& comment) const {
	assert(bool{ term_ });
	return term_(s, comment);
}

namespace cost {

double Constant::operator()(const SubTrajectory& /*s*/, std::string& /*comment*/) const {
	return cost;
}

double Constant::operator()(const SolutionSequence& /*s*/, std::string& /*comment*/) const {
	return cost;
}

double Constant::operator()(const WrappedSolution& /*s*/, std::string& /*comment*/) const {
	return cost;
}

PathLength::PathLength(std::vector<std::string> joints) {
	for (auto& j : joints)
		this->joints.emplace(std::move(j), 1.0);
}

double PathLength::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	const auto& traj = s.trajectory();

	if (traj == nullptr || traj->getWayPointCount() == 0)
		return 0.0;

	std::map<const moveit::core::JointModel*, double> weights;
	const auto& first_waypoint = traj->getWayPoint(0);
	for (auto& joint_weight : joints) {
		const moveit::core::JointModel* jm = first_waypoint.getJointModel(joint_weight.first);
		if (jm)
			weights.emplace(jm, joint_weight.second);
	}

	double path_length{ 0.0 };
	for (size_t i = 1; i < traj->getWayPointCount(); ++i) {
		auto& last = traj->getWayPoint(i - 1);
		auto& curr = traj->getWayPoint(i);
		if (joints.empty()) {
			path_length += last.distance(curr);
		} else {
			for (const auto& item : weights) {
				path_length += item.second * last.distance(curr, item.first);
			}
		}
	}
	return path_length;
}

double JointRiemannianCost::operator()(const SubTrajectory& s, std::string& comment) const {
  const auto& traj = s.trajectory();
  if (!traj || traj->getWayPointCount() < 2) {
    comment += "JointRiemannianCost: empty or single waypoint";
    return 0.0;
  }

  // Build mapping from JointModel* to weight using the first waypoint
  std::map<const moveit::core::JointModel*, double> jm_weights;
  const auto& first_waypoint = traj->getWayPoint(0);

  for (const auto& [joint_name, w] : joint_weights_) {
    const moveit::core::JointModel* jm = first_waypoint.getJointModel(joint_name);
    if (!jm)
      continue;
    if (w <= 0.0)
      continue;  // ignore non-positive weights
    jm_weights.emplace(jm, w);
  }

  double cost = 0.0;

  // Iterate over consecutive waypoint pairs
  for (size_t i = 1; i < traj->getWayPointCount(); ++i) {
    const auto& last = traj->getWayPoint(i - 1);
    const auto& curr = traj->getWayPoint(i);

    if (jm_weights.empty()) {
      // Fallback: square the global joint distance (all joints, equal weights)
      const double d = last.distance(curr);  // MoveIt's default joint-space metric
      cost += d * d;
    } else {
      // Diagonal Riemannian metric: sum_j w_j * (Δq_j)^2
      for (const auto& [jm, w] : jm_weights) {
        const double d = last.distance(curr, jm);  // norm for this joint model
        cost += w * d * d;
      }
    }
  }

  comment += "JointRiemannianCost: " + std::to_string(cost);
  return cost;
}

DistanceToReference::DistanceToReference(const moveit_msgs::msg::RobotState& ref, Mode m,
                                         std::map<std::string, double> w)
  : reference(ref), weights(std::move(w)), mode(m) {}

DistanceToReference::DistanceToReference(const std::map<std::string, double>& ref, Mode m,
                                         std::map<std::string, double> w)
  : weights(std::move(w)), mode(m) {
	reference.joint_state.name.reserve(ref.size());
	reference.joint_state.position.reserve(ref.size());

	for (auto& item : ref) {
		reference.joint_state.name.push_back(item.first);
		reference.joint_state.position.push_back(item.second);
	}
	reference.is_diff = true;
}

double DistanceToReference::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	const auto& state = (mode == Mode::END_INTERFACE) ? s.end() : s.start();
	const auto& traj = s.trajectory();

	moveit::core::RobotState ref_state = state->scene()->getCurrentState();
	moveit::core::robotStateMsgToRobotState(reference, ref_state, false);

	std::map<const moveit::core::JointModel*, double> w;
	for (auto& item : weights) {
		const moveit::core::JointModel* jm = ref_state.getJointModel(item.first);
		if (jm)
			w.emplace(jm, item.second);
	}

	auto distance = [this, &ref_state, &w](const moveit::core::RobotState& state) {
		if (weights.empty()) {
			return ref_state.distance(state);
		} else {
			double accumulated = 0.0;
			for (const auto& item : w)
				accumulated += item.second * ref_state.distance(state, item.first);
			return accumulated;
		}
	};

	if (mode == Mode::START_INTERFACE || mode == Mode::END_INTERFACE || (mode == Mode::AUTO && (traj == nullptr))) {
		return distance(state->scene()->getCurrentState());
	} else {
		double accumulated = 0.0;
		for (size_t i = 0; i < traj->getWayPointCount(); ++i)
			accumulated += distance(traj->getWayPoint(i));
		accumulated /= traj->getWayPointCount();
		return accumulated;
	}
}

// DirectionalManipulability::DirectionalManipulability(
//       std::map<std::string, std::string> group_ee,
//       geometry_msgs::msg::Vector3Stamped direction_vec,
//       Space space,                              // TRANSLATION or ROTATION
//       std::map<std::string,double> group_weights, // group-name -> weight (optional)
//       double epsilon,
//       Mode mode)
//     : group_ee_(std::move(group_ee))
// 	, direction_vec_(std::move(direction_vec))
// 	, dir_frame_(direction_vec_.header.frame_id)
//     , space_(space)
//     , group_weights_(std::move(group_weights))
//     , eps_(epsilon)
//     , mode_(mode) {	
// 		dir_local_ = Eigen::Vector3d(direction_vec_.vector.x, direction_vec_.vector.y, direction_vec_.vector.z);
// 	}

double DirectionalManipulability::operator()(const moveit::task_constructor::SubTrajectory& s, std::string& comment) const {
    const auto& traj  = s.trajectory();
    const auto& state = (mode_ == Mode::END_INTERFACE) ? s.end() : s.start();
    if (!state || !state->scene()){
		comment = "DirectionalManipulability: no valid state";
		return std::numeric_limits<double>::infinity();
	}

	Eigen::Vector3d u_world = dir_local_;
	if (dir_frame_ != "world" && dir_frame_ != "base") {
		const auto& scene = state->scene();
		if (!scene->knowsFrameTransform(dir_frame_)) {
		comment = "unknown direction_frame: " + dir_frame_;
		return std::numeric_limits<double>::infinity();
		}
		u_world = scene->getFrameTransform(dir_frame_).linear() * u_world;
	}
	u_world.normalize();

    if (mode_ == Mode::START_INTERFACE || mode_ == Mode::END_INTERFACE || (mode_ == Mode::AUTO && !traj)) {
      comment = "1/mean_lin_manip";
	  double m = evalState(state->scene()->getCurrentState(), u_world);
	  comment += " = " + std::to_string(m);
      return m;
    }

    if (traj->getWayPointCount() == 0) return 0.0;

    // Average the per-waypoint cost (1 / weighted_mean(m_dir)) over the trajectory
    double acc_cost = 0.0;
    for (size_t i = 0; i < traj->getWayPointCount(); ++i)
      acc_cost += evalState(traj->getWayPoint(i), u_world);
    comment = "1/mean_ang_manip";
	double m = acc_cost / static_cast<double>(traj->getWayPointCount());
	comment += " = " + std::to_string(m);
    return m;
  }

  // Evaluate cost at a single robot state.
  // Steps:
  //   1) resolve direction to world/base
  //   2) for each (group, ee):
  //        - compute 6xN Jacobian, take linear (top 3 rows) or angular (bottom 3)
  //        - m_dir = sqrt( u^T (Js Js^T) u )
  //   3) weighted mean of m_dir across groups
  //   4) cost = 1 / max(mean_m, eps)
  double DirectionalManipulability::evalState(const moveit::core::RobotState& state_in, const Eigen::Vector3d& u_world) const {
    moveit::core::RobotState state = state_in;

    // 2) Accumulate weighted manipulability over groups
    double wsum = 0.0;
    double msum = 0.0;
    size_t used = 0;

    for (const auto& item : group_ee_) {
	  const std::string& group = item.first;
	  const std::string& ee    = item.second;
      const auto* jmg  = state.getJointModelGroup(group);
      const auto* link = state.getLinkModel(ee);
      if (!jmg || !link) continue;

      // group weight
      double w = 1.0;
      if (!group_weights_.empty()) {
        auto it = group_weights_.find(group);
        w = (it != group_weights_.end()) ? it->second : 0.0;  // unspecified groups ignored when map provided
      }
      if (w == 0.0) continue;

      // Jacobian at EE origin (set a non-zero reference point if you need a TCP offset)
      Eigen::MatrixXd J; // 6 x N
      state.getJacobian(jmg, link, tcpOffsetEE(group), J);  // <<< CHANGED
      const Eigen::MatrixXd Js = (space_ == Space::TRANSLATION) ? J.topRows(3) : J.bottomRows(3);

      // m_dir = sqrt( u^T (Js Js^T) u )
      const Eigen::Matrix3d JJt = Js * Js.transpose();
      const double m2 = (u_world.transpose() * JJt * u_world).value();
      const double m  = std::sqrt(std::max(0.0, m2));

      if (m > 0.0) {
        msum += w * m;
        wsum += w;
        ++used;
      }
    }

    if (used == 0 || wsum == 0.0) return std::numeric_limits<double>::infinity();

    const double m_mean = msum / wsum;
    return 1.0 / std::max(m_mean, eps_);
}

double ManipulabilitySoftPenalty::operator()(const SubTrajectory& s, 
											std::string& comment) const
{
    // Use the state we have (end if available; else start)
    const auto* iface = s.end() ? s.end() : s.start();
    if (!iface || !iface->scene()) {
      comment = "ManipulabilitySoftPenalty: no valid state";
      return std::numeric_limits<double>::infinity();
    }
    const moveit::core::RobotState& rs = iface->scene()->getCurrentState();

    // Worst-case (min) σ_min across provided groups
    double smin_worst = std::numeric_limits<double>::infinity();
    std::ostringstream oss;
    oss << "σmin: ";

    size_t used = 0;
    for (const auto& kv : group_ee_) {
      const auto* jmg = rs.getJointModelGroup(kv.first);
      const auto* link = rs.getLinkModel(kv.second);
      if (!jmg || !link) continue;

      const double smin = sigmaMinFullJ(rs, jmg, kv.second);
      smin_worst = std::min(smin_worst, smin);
      ++used;
      oss << kv.first << "=" << smin << " ";
    }

    if (!used) {
      comment = "ManipulabilitySoftPenalty: no valid groups";
      return std::numeric_limits<double>::infinity();
    }

    const double pen = sigmaPenalty(smin_worst, sigma_warn_, sigma_crit_, hard_gate_);
    comment = oss.str();
    return weight_ * pen;
}


double ManipulabilityVolumeCost::operator()(const moveit::task_constructor::SubTrajectory& s,
                                            std::string& comment) const {
  const auto& traj = s.trajectory();
  const auto* iface = s.end() ? s.end() : s.start();
  if (!iface || !iface->scene()) { comment = "ManipVol: no state"; return 1e6; }

  auto eval_one = [&](const moveit::core::RobotState& rs)->double {
    double wsum=0.0, acc=0.0; size_t used=0;

    for (const auto& [group, ee_link] : group_ee_) {
      const auto* jmg  = rs.getJointModelGroup(group);
      const auto* link = rs.getLinkModel(ee_link);
      if (!jmg || !link) continue;

      Eigen::MatrixXd J;  // 6xN at group-specific TCP
      const Eigen::Vector3d tcp_off = tcpOffsetEE(group);
      rs.getJacobian(jmg, link, tcp_off, J);

      const Eigen::MatrixXd Jsub = translation_only_ ? J.topRows(3) : J;
      const double logw = logManipulability(Jsub, lambda_);

      const double w = groupWeight(group);
      if (w > 0.0) { acc += w * logw; wsum += w; ++used; }
    }

    if (!used || wsum == 0.0) return 1e6;
    const double mean_logw = acc / wsum;
    const double cost = 1.0 / (std::exp(mean_logw - mu_) + 1e-3);  // smooth, bounded
    return std::min(weight_ * cost, 1e4);
  };

  double out = 0.0;
  if (traj && traj->getWayPointCount() > 0) {
    double sum = 0.0;
    for (size_t i=0; i<traj->getWayPointCount(); ++i)
      sum += eval_one(traj->getWayPoint(i));
    out = sum / static_cast<double>(traj->getWayPointCount());
    comment = "ManipVol(avg)";
  } else {
    out = eval_one(iface->scene()->getCurrentState());
    comment = "ManipVol(state)";
  }
  return out;
}



double WeightedSumCost::operator()(const moveit::task_constructor::SubTrajectory& s,
                    std::string& comment) const
{
    if (terms_.empty()) { comment += "WeightedSumCost: no terms"; return 0.0; }

    double total = 0.0;
    std::ostringstream oss;
    oss << "WeightedSum: ";

    for (const auto& tw : terms_) {
      if (!tw.first) continue;
      std::string cmt;
      const double c = (*(tw.first))(s, cmt);
      total += tw.second * c;
      if (!cmt.empty()) oss << "[" << cmt << "] ";
    }

    comment += oss.str();
    return total;
}

double TrajectoryDuration::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	return s.trajectory() ? s.trajectory()->getDuration() : 0.0;
}

LinkMotion::LinkMotion(std::string link) : link_name{ std::move(link) } {}

double LinkMotion::operator()(const SubTrajectory& s, std::string& comment) const {
	const auto& traj{ s.trajectory() };

	if (traj == nullptr || traj->getWayPointCount() == 0)
		return 0.0;

	if (!traj->getWayPoint(0).knowsFrameTransform(link_name)) {
		comment = fmt::format("LinkMotionCost: frame '{}' unknown in trajectory", link_name);
		return std::numeric_limits<double>::infinity();
	}

	double distance{ 0.0 };
	Eigen::Vector3d position{ traj->getWayPoint(0).getFrameTransform(link_name).translation() };
	for (size_t i{ 1 }; i < traj->getWayPointCount(); ++i) {
		const auto& new_position{ traj->getWayPoint(i).getFrameTransform(link_name).translation() };
		distance += (new_position - position).norm();
		position = new_position;
	}
	return distance;
}

LinkMotionSum::LinkMotionSum(std::vector<std::string> links, std::vector<Eigen::Isometry3d> offsets) 
	: link_names{ std::move(links) }, offsets{ std::move(offsets) }
	{
		// Fill missing offsets with identity if fewer than links
		if (offsets.size() < link_names.size())
			offsets.resize(link_names.size(), Eigen::Isometry3d::Identity());
	}

double LinkMotionSum::operator()(const SubTrajectory& s, std::string& comment) const {
	const auto& traj = s.trajectory();

	if (!traj || traj->getWayPointCount() == 0)
		return 0.0;

	double total_distance = 0.0;

	for (size_t l = 0; l < link_names.size(); ++l) {
		const std::string& link_name = link_names[l];
		const Eigen::Isometry3d& offset = offsets[l];

		if (!traj->getWayPoint(0).knowsFrameTransform(link_name)) {
			comment += fmt::format("LinkMotionSum: frame '{}' unknown in trajectory\n", link_name);
			total_distance += std::numeric_limits<double>::infinity();
			continue;
		}

		Eigen::Vector3d prev =
			(traj->getWayPoint(0).getFrameTransform(link_name) * offset).translation();

		for (size_t i = 1; i < traj->getWayPointCount(); ++i) {
			Eigen::Vector3d curr =
				(traj->getWayPoint(i).getFrameTransform(link_name) * offset).translation();
			total_distance += (curr - prev).norm();
			prev = curr;
		}
	}

	return total_distance*10;  // Scale the total distance to be aligned with the cost function's expected range
}

Clearance::Clearance(bool with_world, bool cumulative, std::string group_property, Mode mode)
  : with_world{ with_world }
  , cumulative{ cumulative }
  , group_property{ std::move(group_property) }
  , mode{ mode }
  , distance_to_cost{ [](double d) { return 1.0 / (d + 1e-5); } } {}

double Clearance::operator()(const SubTrajectory& s, std::string& comment) const {
	static const std::string PREFIX{ "Clearance: " };

	collision_detection::DistanceRequest request;
	request.type =
	    cumulative ? collision_detection::DistanceRequestType::SINGLE : collision_detection::DistanceRequestType::GLOBAL;

	const auto& state{ (mode == Mode::END_INTERFACE) ? s.end() : s.start() };

	// prefer interface state property over stage property to find group_name
	// TODO: This pattern is general enough to justify its own interface (in the properties?).
	auto& state_properties{ state->properties() };
	auto& stage_properties{ s.creator()->properties() };
	request.group_name = state_properties.hasProperty(group_property) ?
	                         state_properties.get<std::string>(group_property) :
	                         stage_properties.get<std::string>(group_property);

	// look at all forbidden collisions involving group_name
	request.enableGroup(state->scene()->getRobotModel());
	request.acm = &state->scene()->getAllowedCollisionMatrix();

	// compute relevant distance data for state & robot
	auto check_distance{ [=](const InterfaceState* state, const moveit::core::RobotState& robot) {
		collision_detection::DistanceResult result;
		if (with_world)
			state->scene()->getCollisionEnv()->distanceRobot(request, result, robot);
		else
			state->scene()->getCollisionEnv()->distanceSelf(request, result, robot);

		if (result.minimum_distance.distance <= 0) {
			return result.minimum_distance;
		}

		if (cumulative) {
			double distance{ 0.0 };
			for (const auto& distance_of_pair : result.distances) {
				assert(distance_of_pair.second.size() == 1);
				distance += distance_of_pair.second[0].distance;
			}
			result.minimum_distance.distance = distance;
		}

		return result.minimum_distance;
	} };

	auto collision_comment = [=](const auto& distance) {
		return fmt::format(PREFIX + "allegedly valid solution collides between '{}' and '{}'", distance.link_names[0],
		                   distance.link_names[1]);
	};

	double distance{ 0.0 };

	if (mode == Mode::START_INTERFACE || mode == Mode::END_INTERFACE ||
	    (mode == Mode::AUTO && s.trajectory() == nullptr)) {
		auto distance_data{ check_distance(state, state->scene()->getCurrentState()) };
		if (distance_data.distance < 0) {
			comment = collision_comment(distance_data);
			return std::numeric_limits<double>::infinity();
		}
		distance = distance_data.distance;
		if (!cumulative)
			comment = fmt::format(PREFIX + "distance {} between '{}' and '{}'", distance, distance_data.link_names[0],
			                      distance_data.link_names[1]);
		else
			comment = fmt::format(PREFIX + "cumulative distance {}", distance);
	} else {  // check trajectory
		for (size_t i = 0; i < s.trajectory()->getWayPointCount(); ++i) {
			auto distance_data = check_distance(state, s.trajectory()->getWayPoint(i));
			if (distance_data.distance < 0) {
				comment = collision_comment(distance_data);
				return std::numeric_limits<double>::infinity();
			}
			distance += distance_data.distance;
		}
		distance /= s.trajectory()->getWayPointCount();
		comment = fmt::format(PREFIX + "average{} distance: {}", (cumulative ? " cumulative" : ""), distance);
	}

	return distance_to_cost(distance);
}
}  // namespace cost
}  // namespace task_constructor
}  // namespace moveit

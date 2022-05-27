#ifndef _VDP_TAG_DYNAMICS_HPP_
#define _VDP_TAG_DYNAMICS_HPP_

#include <oppt/opptCore/core.hpp>

namespace oppt {
Vector2f vdpDynamics(const Vector2f &targetPos, const FloatType &mu) {
	return Vector2f(mu * (targetPos.x() - targetPos.x() * targetPos.x() * targetPos.x() / 3.0 - targetPos.y()), targetPos.x() / mu);
	//return VectorFloat({mu * (x - std::pow(x, 3.0) / 3.0 - y), x / mu});
}

Vector2f rk4Step(const Vector2f &targetPos,
                 const FloatType &mu,
                 const FloatType &dt,
                 const FloatType &dtHalf,
                 const FloatType &dtDiv6) {
	Vector2f k1 = vdpDynamics(targetPos, mu);
	Vector2f k2 = vdpDynamics(targetPos + dtHalf * k1, mu);
	Vector2f k3 = vdpDynamics(targetPos + dtHalf * k2, mu);
	Vector2f k4 = vdpDynamics(targetPos + dt * k3, mu);
	return targetPos + dtDiv6 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

Vector2f computeTargetPosDeterministic(const Vector2f &currentTargetPos,
                                       const int &steps,
                                       const FloatType &mu,
                                       const FloatType &dt,
                                       const FloatType &dtHalf,
                                       const FloatType &dtDiv6) {
	//VectorFloat nextTargetPos = toStdVec(currentTargetPos);
	Vector2f nextTargetPos(currentTargetPos);
	for (int i = 0; i != steps; ++i) {
		nextTargetPos = rk4Step(nextTargetPos, mu, dt, dtHalf, dtDiv6);
	}

	return nextTargetPos;
}

// Agent movement
/**bool get_line_intersection(FloatType p0_x, FloatType p0_y, FloatType p1_x, FloatType p1_y,
                           FloatType p2_x, FloatType p2_y, FloatType p3_x, FloatType p3_y, Vector2f &interesection) {
	FloatType s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

	FloatType s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		interesection[0] = p0_x + ((t - 1e-4) * s1_x);
		interesection[1] = p0_y + ((t - 1e-4) * s1_y);
		return true;
	}

	return false; // No collision
}*/

FloatType cross(const Vector2f &a, const Vector2f &b) {
	return a.x() * b.y() - b.x() * a.y();
}

Vector2f barrierStop(const Vector2f &agentPos,
                     const Vector2f &nextAgentPos,
                     const std::vector<Vector2f> &cardinals) {

	float shortest_u = 1.0f + 2 * std::numeric_limits<float>::epsilon();
	Vector2f q = agentPos;
	Vector2f s = nextAgentPos;
	for (auto &dir : cardinals) {
		Vector2f p = 0.2f * dir;
		Vector2f r = 2.8f * dir;

		float rxs = cross(r, s);
		if (rxs == 0.0f) {
			continue;
		} else {
			Vector2f qmp = q - p;
			float u = cross(qmp, r) / rxs;
			float t = cross(qmp, s) / rxs;
			if (0.0f <= u && u < shortest_u && 0.0f <= t && t <= 1.0f) {
				shortest_u = u;
			}
		}
	}

	return agentPos + (shortest_u - 2 * std::numeric_limits<float>::epsilon()) * nextAgentPos;
}

}

#endif
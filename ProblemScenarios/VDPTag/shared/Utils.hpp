#ifndef _VDP_TAG_UTILS_HPP_
#define _VDP_TAG_UTILS_HPP_

namespace oppt {

float rectifyAngle(const float &angle) {
	return angle - (ceilf((angle + M_PI) / (2 * M_PI)) - 1) * 2 * M_PI;
}

float angleTo(const Vector2f &from, const Vector2f &to) {
	return rectifyAngle(atan2f(from.x() * to.y() - from.y() * to.x(), from.x() * to.x() + from.y() * to.y()));
}


int activeBeam(const Vector2f &relativePos) {
	float angle = angleTo(Vector2f(1.0f, 0.0f), relativePos);
	while (angle <= 0.0f) {
		angle += 2 * M_PI;
	}
	size_t x = static_cast<size_t>(lround(ceilf(8 * angle / (2 * M_PI))) - 1);
	return std::max(static_cast<size_t>(0), std::min(static_cast<size_t>(7), x));
}
}

#endif
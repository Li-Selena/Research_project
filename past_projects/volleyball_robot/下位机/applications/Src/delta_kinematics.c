#include <math.h>
#include <stdint.h>
#include "control_config.h"
#include "delta_clac.h"

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

uint8_t DeltaInverseKinematics(float x, float y, float z, float joint_deg[3])
{
    const float phases[3] = {0.0f, 2.0f * CONTROL_PI / 3.0f,
                             4.0f * CONTROL_PI / 3.0f};
    const float radial_offset = DELTA_PLATFORM_RADIUS_M - DELTA_BASE_RADIUS_M;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z) || z < 0.0f) return 0;
    for (uint8_t i = 0; i < 3; ++i) {
        float cp = cosf(phases[i]);
        float sp = sinf(phases[i]);
        float radial = x * cp + y * sp + radial_offset;
        float tangent = -x * sp + y * cp;
        float c = radial * radial + tangent * tangent + z * z +
                  DELTA_UPPER_ARM_M * DELTA_UPPER_ARM_M -
                  DELTA_LOWER_ARM_M * DELTA_LOWER_ARM_M;
        float radius = sqrtf(radial * radial + z * z);
        float ratio;
        float angle;

        if (radius < 1.0e-6f) return 0;
        ratio = c / (2.0f * DELTA_UPPER_ARM_M * radius);
        if (ratio < -1.00001f || ratio > 1.00001f) return 0;
        ratio = clampf(ratio, -1.0f, 1.0f);
        angle = atan2f(z, radial) - acosf(ratio);
        joint_deg[i] = angle * 180.0f / CONTROL_PI;
        if (!isfinite(joint_deg[i]) || joint_deg[i] < DELTA_JOINT_MIN_DEG ||
            joint_deg[i] > DELTA_JOINT_MAX_DEG) return 0;
    }
    return 1;
}

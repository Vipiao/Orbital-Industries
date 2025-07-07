// MassInertiaCalculator.cpp
#include "MassInertiaCalculator.h"

// ===== HELPER FUNCTION IMPLEMENTATIONS =====

glm::dmat3 MassInertiaCalculator::applyParallelAxisTheorem(
    const glm::dmat3& localTensor,
    double mass,
    const glm::dvec3& displacement) {
    
    // Parallel axis theorem: I_new = I_local + m*(d²*I - r⊗r)
    double distanceSquared = glm::dot(displacement, displacement);
    
    // Create r⊗r (outer product)
    glm::dmat3 outerProduct(0.0);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            outerProduct[i][j] = displacement[i] * displacement[j];
        }
    }
    
    // Create d²*I (identity matrix scaled by distance squared)
    glm::dmat3 scaledIdentity(0.0);
    scaledIdentity[0][0] = scaledIdentity[1][1] = scaledIdentity[2][2] = distanceSquared;
    
    return localTensor + mass * (scaledIdentity - outerProduct);
}
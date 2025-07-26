// MassInertiaCalculator.h
#pragma once

#include <glm/glm.hpp>

class MassInertiaCalculator {
public:
    // Helper structs for data extraction - scoped to avoid naming conflicts
    struct ObjectData {
        glm::dvec3 position;
        double mass;
        double localInertia;  // For scalar inertia (spheres/balls)
        
        ObjectData(const glm::dvec3& pos, double m, double inertia)
            : position(pos), mass(m), localInertia(inertia) {}
    };
    
    struct TensorObjectData {
        glm::dvec3 position;
        double mass;
        glm::dmat3 localTensor;  // For full tensor inertia (complex shapes)
        
        TensorObjectData(const glm::dvec3& pos, double m, const glm::dmat3& tensor)
            : position(pos), mass(m), localTensor(tensor) {}
    };

    // ===== SCALAR INERTIA INTERFACE (most efficient for spherical objects) =====
    
    template<typename Container, typename DataExtractor>
    static void calculateScalarInertia(
        const Container& container,
        DataExtractor getData,
        double* outTotalMass,
        glm::dvec3* outCenterOfMass,
        double* outScalarInertia);
    
    template<typename Container, typename DataExtractor>
    static void calculateScalarInertiaIncremental(
        const Container& newContainer,
        DataExtractor getData,
        double* inOutTotalMass,
        glm::dvec3* inOutCenterOfMass,
        double* inOutScalarInertia);
    
    // ===== TENSOR INERTIA INTERFACE (for complex shapes) =====
    
    template<typename Container, typename TensorDataExtractor>
    static void calculateTensorInertia(
        const Container& container,
        TensorDataExtractor getData,
        double* outTotalMass,
        glm::dvec3* outCenterOfMass,
        glm::dmat3* outInertiaTensor);
    
    template<typename Container, typename TensorDataExtractor>
    static void calculateTensorInertiaIncremental(
        const Container& newContainer,
        TensorDataExtractor getData,
        double* inOutTotalMass,
        glm::dvec3* inOutCenterOfMass,
        glm::dmat3* inOutInertiaTensor);

private:
    // Helper function for tensor calculations
    static glm::dmat3 applyParallelAxisTheorem(
        const glm::dmat3& localTensor,
        double mass,
        const glm::dvec3& displacement);
};

// ===== TEMPLATE IMPLEMENTATIONS (must be in header) =====

template<typename Container, typename DataExtractor>
void MassInertiaCalculator::calculateScalarInertia(
    const Container& container,
    DataExtractor getData,
    double* outTotalMass,
    glm::dvec3* outCenterOfMass,
    double* outScalarInertia) {
    
    if (container.empty()) {
        if (outTotalMass) *outTotalMass = 0.0;
        if (outCenterOfMass) *outCenterOfMass = glm::dvec3(0.0);
        if (outScalarInertia) *outScalarInertia = 0.0;
        return;
    }
    
    // Calculate total mass and weighted center of mass
    glm::dvec3 weightedSum(0.0);
    double totalMass = 0.0;
    
    for (const auto& item : container) {
        ObjectData data = getData(item);
        totalMass += data.mass;
        weightedSum += data.position * data.mass;
    }
    
    glm::dvec3 centerOfMass = weightedSum / totalMass;
    
    // Calculate scalar inertia with proper mass weighting
    double scalarInertia = 0.0;
    for (const auto& item : container) {
        ObjectData data = getData(item);
        glm::dvec3 displacement = data.position - centerOfMass;
        double distanceSquared = glm::dot(displacement, displacement);
        // Parallel axis theorem for scalar: I_total = I_local + m * d²
        scalarInertia += data.localInertia + data.mass * distanceSquared;
    }
    
    // Set outputs
    if (outTotalMass) *outTotalMass = totalMass;
    if (outCenterOfMass) *outCenterOfMass = centerOfMass;
    if (outScalarInertia) *outScalarInertia = scalarInertia;
}

template<typename Container, typename DataExtractor>
void MassInertiaCalculator::calculateScalarInertiaIncremental(
    const Container& newContainer,
    DataExtractor getData,
    double* inOutTotalMass,
    glm::dvec3* inOutCenterOfMass,
    double* inOutScalarInertia) {
    
    if (newContainer.empty()) {
        return; // No change
    }
    
    // Get current state
    double existingMass = inOutTotalMass ? *inOutTotalMass : 0.0;
    glm::dvec3 existingCM = inOutCenterOfMass ? *inOutCenterOfMass : glm::dvec3(0.0);
    double existingInertia = inOutScalarInertia ? *inOutScalarInertia : 0.0;
    
    // Calculate properties of new objects
    double newMass, newInertia;
    glm::dvec3 newCM;
    calculateScalarInertia(newContainer, getData, &newMass, &newCM, &newInertia);
    
    // Combine masses
    double totalMass = existingMass + newMass;

    // Handle case where total mass becomes zero or negative
    if (totalMass <= 0.0) {
        if (inOutTotalMass) *inOutTotalMass = 0.0;
        if (inOutCenterOfMass) *inOutCenterOfMass = glm::dvec3(0.0);
        if (inOutScalarInertia) *inOutScalarInertia = 0.0;
        return;
    }
    
    // Combine center of mass
    glm::dvec3 combinedCM = (existingCM * existingMass + newCM * newMass) / totalMass;
    
    // For scalar inertia, use parallel axis theorem directly
    glm::dvec3 existingShift = existingCM - combinedCM;
    double existingShiftSquared = glm::dot(existingShift, existingShift);
    double adjustedExistingInertia = existingInertia + existingMass * existingShiftSquared;
    
    glm::dvec3 newShift = newCM - combinedCM;
    double newShiftSquared = glm::dot(newShift, newShift);
    double adjustedNewInertia = newInertia + newMass * newShiftSquared;
    
    double combinedInertia = adjustedExistingInertia + adjustedNewInertia;
    
    // Set outputs
    if (inOutTotalMass) *inOutTotalMass = totalMass;
    if (inOutCenterOfMass) *inOutCenterOfMass = combinedCM;
    if (inOutScalarInertia) *inOutScalarInertia = combinedInertia;
}

template<typename Container, typename TensorDataExtractor>
void MassInertiaCalculator::calculateTensorInertia(
    const Container& container,
    TensorDataExtractor getData,
    double* outTotalMass,
    glm::dvec3* outCenterOfMass,
    glm::dmat3* outInertiaTensor) {
    
    if (container.empty()) {
        if (outTotalMass) *outTotalMass = 0.0;
        if (outCenterOfMass) *outCenterOfMass = glm::dvec3(0.0);
        if (outInertiaTensor) *outInertiaTensor = glm::dmat3(0.0);
        return;
    }
    
    // First pass: calculate total mass and center of mass
    glm::dvec3 weightedSum(0.0);
    double totalMass = 0.0;
    
    for (const auto& item : container) {
        TensorObjectData data = getData(item);  // Should return TensorObjectData
        totalMass += data.mass;
        weightedSum += data.position * data.mass;
    }
    
    glm::dvec3 centerOfMass = weightedSum / totalMass;
    
    // Second pass: calculate inertia tensor with parallel axis theorem
    glm::dmat3 totalTensor(0.0);
    for (const auto& item : container) {
        TensorObjectData data = getData(item);
        glm::dvec3 displacement = data.position - centerOfMass;
        glm::dmat3 transformedTensor = applyParallelAxisTheorem(
            data.localTensor, data.mass, displacement);
        totalTensor += transformedTensor;
    }
    
    // Set outputs
    if (outTotalMass) *outTotalMass = totalMass;
    if (outCenterOfMass) *outCenterOfMass = centerOfMass;
    if (outInertiaTensor) *outInertiaTensor = totalTensor;
}

template<typename Container, typename TensorDataExtractor>
void MassInertiaCalculator::calculateTensorInertiaIncremental(
    const Container& newContainer,
    TensorDataExtractor getData,
    double* inOutTotalMass,
    glm::dvec3* inOutCenterOfMass,
    glm::dmat3* inOutInertiaTensor) {
    
    if (newContainer.empty()) {
        return; // No change
    }
    
    // Get current state
    double existingMass = inOutTotalMass ? *inOutTotalMass : 0.0;
    glm::dvec3 existingCM = inOutCenterOfMass ? *inOutCenterOfMass : glm::dvec3(0.0);
    glm::dmat3 existingTensor = inOutInertiaTensor ? *inOutInertiaTensor : glm::dmat3(0.0);
    
    // Calculate properties of new objects
    double newMass;
    glm::dvec3 newCM;
    glm::dmat3 newTensor;
    calculateTensorInertia(newContainer, getData, &newMass, &newCM, &newTensor);
    
    if (existingMass <= 0.0) {
        // No existing mass, just use new values
        if (inOutTotalMass) *inOutTotalMass = newMass;
        if (inOutCenterOfMass) *inOutCenterOfMass = newCM;
        if (inOutInertiaTensor) *inOutInertiaTensor = newTensor;
        return;
    }
    
    // Combine masses
    double totalMass = existingMass + newMass;

    // Handle case where total mass becomes zero or negative
    if (totalMass <= 0.0) {
        if (inOutTotalMass) *inOutTotalMass = 0.0;
        if (inOutCenterOfMass) *inOutCenterOfMass = glm::dvec3(0.0);
        if (inOutInertiaTensor) *inOutInertiaTensor = glm::dmat3(0.0);
        return;
    }
    
    // Combine center of mass
    glm::dvec3 combinedCM = (existingCM * existingMass + newCM * newMass) / totalMass;
    
    // Adjust tensors for center of mass shift and combine
    glm::dvec3 existingShift = existingCM - combinedCM;
    glm::dmat3 adjustedExistingTensor = applyParallelAxisTheorem(
        existingTensor, existingMass, existingShift);
    
    glm::dvec3 newShift = newCM - combinedCM;
    glm::dmat3 adjustedNewTensor = applyParallelAxisTheorem(
        newTensor, newMass, newShift);
    
    glm::dmat3 combinedTensor = adjustedExistingTensor + adjustedNewTensor;
    
    // Set outputs
    if (inOutTotalMass) *inOutTotalMass = totalMass;
    if (inOutCenterOfMass) *inOutCenterOfMass = combinedCM;
    if (inOutInertiaTensor) *inOutInertiaTensor = combinedTensor;
}
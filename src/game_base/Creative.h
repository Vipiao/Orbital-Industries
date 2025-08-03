// Creative.h
#pragma once

#include "Mode.h"
#include <glm/glm.hpp>

/**
 * @brief Creative mode implementation with block placement/removal and force application
 */
class Creative : public Mode {
public:
    Creative(GameBase* gameBase);
    virtual ~Creative() = default;
    
    virtual void processInputs() override;
    virtual void physics() override;
    
private:
    // Input flags
    bool doCreate = false;
    bool doRemove = false; 
    bool doForce = false;
    bool doTrackSpeed = false;
    double forceMultiplier = 1.0;
    double m_moveSpeed = 0.05;
    
    // Helper methods
    void addGridBlock(class Grid* grid, int x, int y, int z);
    void removeGridBlock(class Grid* grid, int x, int y, int z);
    void applyDragForces();
    void processInputLogic();
};
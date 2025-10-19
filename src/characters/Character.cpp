// Character.cpp
#include "Character.h"

int Character::s_nextId = 0;

Character::Character(PhysicsEngine* physics, GraphicsEngine* graphics,
                     JobManager* jobManager, TimeHandler* timeHandler)
    : m_physics(physics)
    , m_graphics(graphics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
    , m_rigidBody(nullptr)
    //, m_centerOfMass(0.0, 0.0, 0.0)
    , m_uniqueId(s_nextId++)
{
}

Character::~Character() {
    
}
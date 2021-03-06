#include "XkPhysicsWorld.h"

namespace Xk
{

XkPhysicsWorld::XkPhysicsWorld()
{

}

XkPhysicsWorld::~XkPhysicsWorld()
{

}

bool XkPhysicsWorld::initialize()
{
    return initializeImpl();
}

void XkPhysicsWorld::uninitialize()
{
    uninitializeImpl();
}

bool XkPhysicsWorld::step(float fTime)
{
    return stepImpl(fTime);
}

} //namespace Xk
#ifndef XK_PHYSICS_WORLD_H
#define XK_PHYSICS_WORLD_H

#include "XkSingleton.h"

#if XK_PHYSICS_LIB == XK_PHYSICS_LIB_HAVOK
#include "XkPhysicsWorld_Havok.h"
#elif XK_PHYSICS_LIB == XK_PHYSICS_LIB_PHYSX
#include "XkPhysicsWorld_PhysX.h"
#endif

namespace Xk
{

class XkPhysicsWorld : public XkPhysicsWorldImpl
{
public:
    XkPhysicsWorld();
    ~XkPhysicsWorld();

public:
    bool initialize();
    void uninitialize();
    bool step(float fTime);
};

typedef Xk::Singleton<XkPhysicsWorld> World;

} //namespace Xk

#endif
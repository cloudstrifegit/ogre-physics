#ifndef XK_PHYSICS_WORLD_PHYSX_H
#define XK_PHYSICS_WORLD_PHYSX_H

#if XK_PHYSICS_LIB == XK_PHYSICS_LIB_HAVOK

class hkMemoryRouter;
class hkJobThreadPool;
class hkJobQueue;
class hkpWorld;

namespace Xk
{

class XkPhysicsWorldImpl
{
public:
        XkPhysicsWorldImpl();
        ~XkPhysicsWorldImpl();

        static void errorReport(const char* msg, void* userArgGivenToInit);

public:
        bool initializeImpl();
        void uninitializeImpl();
        bool stepImpl(float fTime);

private:
        bool createPhysicsWorld();

private:
        hkMemoryRouter*                m_memoryRouter;
        hkJobThreadPool*        m_threadPool;
        hkJobQueue*                        m_jobQueue;
        int                                        m_nThreadUsed;
        hkpWorld*                        m_physicsWorld;

#ifdef PHYSICS_SYSTEM_HAVOK_USE_VDB
        hkpPhysicsContext*                        m_physicsContext;
        hkVisualDebugger*                        m_vdb;
#endif

        float                        m_accumulator;
        float                        m_stepSize;
};

} //namespace Xk

#endif //#if XK_PHYSICS_LIB == XK_PHYSICS_LIB_HAVOK

#endif
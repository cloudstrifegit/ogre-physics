#ifndef XK_CONFIG_H
#define XK_CONFIG_H

//是否启用DebugString
#define XK_ENABLE_DEBUG_STR 1

//是否启用Debug Log
#define XK_ENABLE_LOG_DEBUG 1

//是否启用Warning Log
#define XK_ENABLE_LOG_WARNING 1

//是否启用Error Log
#define XK_ENABLE_LOG_ERROR 1

//是否启用Dump Log
#define XK_ENABLE_LOG_DUMP 1

//使用哪一个物理引擎
#define XK_PHYSICS_LIB_PHYSX 0
#define XK_PHYSICS_LIB_HAVOK 1

//默认使用Havok，Linux平台下使用PhysX
#define XK_PHYSICS_LIB XK_PHYSICS_LIB_HAVOK
#if defined(LINUX) || defined(_LINUX)
	#define XK_PHYSICS_LIB XK_PHYSICS_LIB_PHYSX
#endif

#define XK_HAVOK_ENABLE_VDB 0

#endif
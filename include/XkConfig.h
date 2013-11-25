#ifndef XK_CONFIG_H
#define XK_CONFIG_H

//�Ƿ�����DebugString
#define XK_ENABLE_DEBUG_STR 1

//�Ƿ�����Debug Log
#define XK_ENABLE_LOG_DEBUG 1

//�Ƿ�����Warning Log
#define XK_ENABLE_LOG_WARNING 1

//�Ƿ�����Error Log
#define XK_ENABLE_LOG_ERROR 1

//�Ƿ�����Dump Log
#define XK_ENABLE_LOG_DUMP 1

//ʹ����һ����������
#define XK_PHYSICS_LIB_PHYSX 0
#define XK_PHYSICS_LIB_HAVOK 1

//Ĭ��ʹ��Havok��Linuxƽ̨��ʹ��PhysX
#define XK_PHYSICS_LIB XK_PHYSICS_LIB_HAVOK
#if defined(LINUX) || defined(_LINUX)
	#define XK_PHYSICS_LIB XK_PHYSICS_LIB_PHYSX
#endif

#define XK_HAVOK_ENABLE_VDB 0

#endif
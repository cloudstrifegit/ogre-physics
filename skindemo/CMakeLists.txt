cmake_minimum_required(VERSION 2.8)

PROJECT(skindemo)
SET(havokinc $ENV{SDK_HOME}/hk2013_1_0_r1/Source)
SET(ogreinc $ENV{OGRE_HOME}/include)
SET(pocoinc $ENV{SDK_HOME}/poco/include)

SET(havoklib 
	$ENV{SDK_HOME}/hk2013_1_0_r1/Lib/win32_vs2010
)

INCLUDE_DIRECTORIES(
	${havokinc}
	${ogreinc}
	${pocoinc}
	${CMAKE_SOURCE_DIR}/../include
)

LINK_DIRECTORIES(
	${havoklib}
	$ENV{OGRE_HOME}/lib/release
	$ENV{OGRE_HOME}/lib/release/opt
	$ENV{SDK_HOME}/poco/lib
)

SET(source	XkPhysicsWorld.h
			XkPhysicsWorld_Havok.h
			XkPhysicsWorld_PhysX.h
			XkPhysicsWorld.cpp
			XkPhysicsWorld_Havok.cpp
			main.cpp
)

ADD_EXECUTABLE(skindemo ${source})

TARGET_LINK_LIBRARIES( skindemo hkBase.lib
								hkCompat.lib
								hkSceneData.lib
								hkSerialize.lib
								hkInternal.lib
								hkGeometryUtilities.lib
								hkVisualize.lib
								hkcdInternal.lib
								hkcdCollide.lib
								hkpCollide.lib
								hkpConstraint.lib
								hkpConstraintSolver.lib
								hkpDynamics.lib
								hkpInternal.lib
								hkpUtilities.lib
								hkpVehicle.lib
								OgreMain.lib
								OgreOverlay.lib
								OgrePaging.lib
								OgreRTShaderSystem.lib
								OgreTerrain.lib
								OgreVolume.lib
								OIS.lib
								Plugin_BSPSceneManager.lib
								Plugin_CgProgramManager.lib
								Plugin_OctreeSceneManager.lib
								Plugin_OctreeZone.lib
								Plugin_ParticleFX.lib
								Plugin_PCZSceneManager.lib
								RenderSystem_Direct3D9.lib
								RenderSystem_Direct3D11.lib
)
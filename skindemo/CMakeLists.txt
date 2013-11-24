cmake_minimum_required(VERSION 2.8)

PROJECT(skindemo)
SET(havokinc $ENV{SNH_HOME}/hk2013_1_0_r1/Source)
SET(ogreinc $ENV{OGRE_HOME}/include)
SET(pocoinc $ENV{SNH_HOME}/poco/include)

SET(havoklib 
	$ENV{SNH_HOME}/hk2013_1_0_r1/Lib/win32_vs2010
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
	$ENV{SNH_HOME}/poco/lib
)

SET(source main.cpp
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
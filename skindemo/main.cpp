#include "XkPhysicsWorld.h"
#include <stdio.h>

int main(int argc, char** argv)
{
	Xk::World::instance().initialize();
	getchar();
	return 0;
}
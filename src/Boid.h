#pragma once
#include "btBulletDynamicsCommon.h"


class Boid
{

public:
//vars
	btRigidBody * body0;
	void Run();



	//not important
	bool drawGizmos = true;

	


//my methods
private:
	
	const btScalar MAX_VELOCITY = 30;
	const btScalar MAX_DISTANCE = 100;
	const btScalar THRUST_FORCE= 20;

	//boid vectors
	btVector3 boid_front;
	btVector3 boid_top;
	btVector3 boid_right;

	//variabels that are updated
	btVector3 position;//boid position
	btScalar mass;
	btVector3 vel;
	btVector3 gravity;
	btTransform trans;
	btVector3 avel;
	btVector3 thrust;
	btVector3 drag;
	btVector3 lift;


	btVector3 dir;



	void LimitVelocity(btScalar _limit);
	void RadialLimit(btScalar _limit);


	void DrawLine1(const btVector3 &from, const btVector3 &to, const btVector3 &c);
	


//my colours
	struct Colour {
		const btVector3 red = btVector3(1, 0, 0);
		const btVector3 green = btVector3(0, 1, 0);
		const btVector3 blue = btVector3(0, 0, 1);
		const btVector3 black = btVector3(0, 0, 0);
		const btVector3 white = btVector3(1, 1, 1);
	} colour;

//basic vectors
	struct Vec {
		const btVector3 forward = btVector3(1, 0, 0);
		const btVector3 up = btVector3(0, 1, 0);
		const btVector3 right = btVector3(0, 0, 1);
		const btVector3 zero = btVector3(0, 0, 0);
	} vec;



};






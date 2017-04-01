#include "Boid.h"
#include "GlutStuff.h"


void DrawLine1(btVector3 &from, btVector3 &to, btVector3 &c) {
	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(c.x(), c.y(), c.z());
	btglVertex3(from.x(), from.y(), from.z());
	btglVertex3(to.x(), to.y(), to.z());
	glEnd();

}


void Boid::Run()
{


	//distance
	float centerDist = body0->getWorldTransform().getOrigin().length();
	btVector3 lookAtCenter = -btVector3(body0->getWorldTransform().getOrigin());//negative pos
	lookAtCenter.normalize();// normalise it
	btVector3 boidPos = btVector3(body0->getWorldTransform().getOrigin());//boid pos
	btVector3 boidForward = btVector3(body0->getWorldTransform().getOrigin()) * btVector3(1, 0, 0);//boid pos
	btVector3 boidUp = btVector3(body0->getWorldTransform().getOrigin()) * btVector3(0, 1, 0);//boid up
																							  //body0->getCenterOfMassTransform()
	boidForward.normalize();//normalised



	//string example;
	//example = std::to_string(centerDist);
	//printf(example.c_str());
	//printf("\n");



	btScalar mass = body0->getInvMass();
	btVector3 vel = body0->getLinearVelocity();
	btVector3 gravity = body0->getGravity();
	btVector3 dir = btVector3(0, 0, 1);
	dir = lookAtCenter;


	// Limit areas
	if ((boidPos.length()>100.0f) && (btDot(-boidPos.normalized(), boidForward)< 0)) {
		dir = btVector3(-boidPos.normalized());
		DrawLine1(boidPos, btVector3(0, 0, 0), btVector3(1, 1, 1));

	}
	else {
		dir = btVector3(0, 0, 0);
		//dir = btVector3(boidForward);
		DrawLine1(boidPos, btVector3(0, 0, 0), btVector3(0, 0, 0));
	}




	btTransform btrans(body0->getOrientation());
	btVector3 top = btrans * btVector3(0, 1, 0);
	btVector3 front = btrans * btVector3(1, 0, 0);
	btVector3 right = btrans * btVector3(0, 0, 1);
	btVector3 dir1 = vel.safeNormalize();
	btVector3 avel = body0->getAngularVelocity();
	btVector3 bthrust = 20.0 * front;
	btVector3 bdrag = -15 * vel;
	btVector3 blift = -1.00 * gravity * vel.length();
	//add up vector later
	//body0->applyTorque(20.0 * front.cross(dir) - 2.0*avel);//forward and dir
	//body0->applyTorque(10.0 * front.cross(dir) - 6.0*avel);//forward and dir
	body0->applyTorque(13.0 * front.cross(dir) - 2.0*avel);
	body0->applyTorque(-6.5 * btVector3(0, 1, 0));//stay up
	body0->applyTorque(10.5 * top.cross(btVector3(0, 1, 0)) - 10 * avel);//left/right tilt



	DrawLine1(boidPos, boidPos + front * 15, btVector3(0, 0, 1));
	DrawLine1(boidPos, boidPos + top * 15, btVector3(0, 1, 0));
	DrawLine1(boidPos, boidPos + right * 15, btVector3(1, 0, 0));


	//limit velocity
	float velocity = body0->getLinearVelocity().length();
	if (velocity < 30.0f) {
		body0->applyCentralForce(bthrust + blift + gravity + bdrag);
	}
	else {
		body0->applyCentralForce(blift + gravity + bdrag);
	}
	//--limit vel!









}



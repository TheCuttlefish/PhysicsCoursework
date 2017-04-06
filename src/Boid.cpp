#include "Boid.h"
#include "GlutStuff.h"


#include <stdio.h> //printf debugging
#include <string.h>
#include <string>



using namespace std;


void Boid::Run(std::vector <Boid> &boids)
{


	//main physics
	{
		position = btVector3(body0->getWorldTransform().getOrigin());//boid pos
		mass = body0->getInvMass();
		vel = body0->getLinearVelocity();
		vel.safeNormalize();//normalise
		gravity = body0->getGravity();
		trans = btTransform(body0->getOrientation());
		avel = body0->getAngularVelocity();
		thrust = THRUST_FORCE * boid_front;
		drag = -10 * vel;
		lift = -1.00 * gravity * vel.length();
		boid_front = trans * vec.forward;
		boid_top = trans * vec.up;
		boid_right = trans * vec.right;
		//add forces
		body0->applyTorque(Separation(boids)+Alignment(boids)+
							10.0 * boid_front.cross(dir*PHYSICS_STRENGTH) - 6.0*avel);
		//body0->applyTorque(10.0 * boid_front.cross(dir) - 6.0*avel);
		body0->applyTorque(-6.5 * vec.up);//stay up
		body0->applyTorque(10.5 * boid_top.cross(vec.up) - 10 * avel);//left/right tilt
		//LimitVelocity(MAX_VELOCITY);//apply velocity

		Cohesion(boids);
		


		if (windForce) {
			body0->applyCentralForce(btVector3(2, 0, 0));
		}



		velocity = body0->getLinearVelocity().length();
		if (velocity < MAX_VELOCITY) {
			body0->applyCentralForce(thrust + lift + gravity + drag);
		}
		else {
			body0->applyCentralForce(lift + gravity + drag);
		}

	}


	RadialLimit(MAX_DISTANCE);
	//
	
	//


	//draw axes
	//DrawLine1(position, position + boid_front * 15, colour.blue);
	//DrawLine1(position, position + boid_top * 15, colour.green);
	//DrawLine1(position, position + boid_right * 15, colour.red);


}


btVector3 Boid::Alignment(std::vector <Boid> &boids) {
	btVector3 aVec = btVector3(0,0,0);


	



	for (auto & boid : boids) {
		//can see
		if (boid.body0 == body0) {

		} else {
			btScalar dist = btDistance(boid.position, position);
			//can see
			if(dist<MAX_ALIGHNMENT_VISIBILITY){//50


				string example;
				example = to_string(boid_front.x());
				//printf(example.c_str());
				//printf("\n");


				if (btDot(boid_front, boid.position - position)<VISIBILITY) {//in fron
					//DrawLine1(position, boid.position, colour.green);
					aVec = aVec + btTransform(boid.body0->getOrientation())*vec.forward;
					aVec = aVec.safeNormalize();
				}
			}

		}


	}


		DrawLine1(position, position + aVec * 30, colour.green);


	return aVec*ALIGNMENT_STRENGHT;// *1
}





btVector3 Boid::Cohesion(std::vector <Boid> &boids) {
	btVector3 cVec = btVector3(0, 0, 0);

	for (auto & boid : boids) {
		//can see
		if (boid.body0 == body0) {

		}
		else {
			btScalar dist = btDistance(boid.position, position);
			//can see
			if (dist<MAX_COHESION_VISIBILITY && dist>MAX_SEPARATION_VISIBILITY) {//30




				if (btDot(boid_front, boid.position - position)<VISIBILITY) {//in front
					//Cohesion
					cVec = cVec + boid.position;
				}
			}
		}


	}

	//get the average position
	
	cVec = cVec.safeNormalize();
	body0->applyCentralForce(cVec*COHESION_STRENGHT);

	//body0->applyTorque(cVec*1);//????5
	//10 is good
	return cVec*COHESION_STRENGHT;
}

btVector3 Boid::Separation(std::vector <Boid> &boids) {
	btVector3 sVec = btVector3(0, 0, 0);

	for (auto & boid : boids) {
		//can see
		if (boid.body0 == body0) {

		}
		else {
			btScalar dist = btDistance(boid.position, position);
			//can see
			if (dist < MAX_SEPARATION_VISIBILITY) {//30
						if (btDot(boid_front, boid.position - position) < VISIBILITY) {//in front
							sVec = sVec + position - boid.position;
						}
			}
		}


	}

	sVec.safeNormalize();
	sVec = sVec*-1;
	body0->applyCentralForce(-sVec*SEPARATION_STRENGHT);
	
		DrawLine1(position, position - sVec * 30, colour.red);
	
	return sVec*SEPARATION_STRENGHT;
}

void Boid::LimitVelocity(btScalar _limit)
{
	float velocity = body0->getLinearVelocity().length();
	if (velocity < _limit) {
		body0->applyCentralForce(thrust + lift + gravity + drag);
	}
	else {
		body0->applyCentralForce(lift + gravity + drag);
	}
}

void Boid::RadialLimit(btScalar _limit)
{
	//in the radius //-----------------------------------------------------0.9 circle and -.9 flocking no order - good
	if ((position.length()>_limit) && (btDot(-position.normalized(), boid_front)< 0.9)) {//0.5
		dir = btVector3(-position.normalized()) ;

		//DrawLine1(position, vec.zero, colour.white);

	}
	else {
		dir = vec.zero;
		//DrawLine1(position, vec.zero, colour.black);
	}
}




void Boid::DrawLine1(const btVector3 &from, const btVector3 &to, const btVector3 &c) {

	if (!drawGizmos)return;

	glLineWidth(0.1f);
	glBegin(GL_LINES);
	glColor3f(c.x(), c.y(), c.z());
	btglVertex3(from.x(), from.y(), from.z());
	btglVertex3(to.x(), to.y(), to.z());
	glEnd();

}

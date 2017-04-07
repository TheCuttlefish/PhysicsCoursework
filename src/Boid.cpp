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
		drag = -DRAG_FORCE * vel;
		lift = -1.00 * gravity * vel.length();
		boid_front = trans * vec.forward;
		boid_top = trans * vec.up;
		boid_right = trans * vec.right;
		//add forces
		body0->applyTorque(20.0 * boid_front.cross(Separation(boids) +
													Alignment(boids) +
													dir*PHYSICS_STRENGTH) - 10.0*avel);
		Cohesion(boids);//--applied only on force (not torque)
		body0->applyTorque(-6.5 * vec.up);//stay up   /-6.5
		body0->applyTorque(20.5 * boid_top.cross(vec.up) - 10 * avel);//left/right tilt
		LimitVelocity(MAX_VELOCITY);//apply velocity limit


		if (windForce) {
			body0->applyCentralForce(btVector3(2, 0, 0));
		}

	}

	//limit the environment
	RadialLimit(MAX_DISTANCE);

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
				if (btDot(boid_front, boid.position - position)>VISIBILITY) {//in fron
					aVec = aVec + btTransform(boid.body0->getOrientation())*vec.forward;
					//show who I'm looking at
					DrawLine1(position, position+(boid.position-position)/2, colour.black);
				}
			}
		}
	}

	if (aVec.length()>1) {
		aVec = aVec.safeNormalize();
	}
		DrawLine1(position, position + aVec * 30, colour.green);


	return aVec*ALIGNMENT_STRENGTH;
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




				if (btDot(boid_front, boid.position - position)>VISIBILITY) {//in front
					//Cohesion
					cVec = cVec + boid.position;
				}
			}
		}


	}

	//get the average position
	if (cVec.length()>1) {
		cVec = cVec.safeNormalize();
	}
	body0->applyCentralForce(cVec*COHESION_STRENGTH);
	return cVec*COHESION_STRENGTH;
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
						if (btDot(boid_front, boid.position - position) > VISIBILITY) {//in front
							sVec = sVec + position - boid.position;
				}
			}
		}


	}
	if (sVec.length()>1) {
		sVec = sVec.safeNormalize();
	}
	
	sVec = sVec*-1;
	body0->applyCentralForce(sVec*SEPARATION_STRENGTH);

	return sVec*SEPARATION_STRENGTH;
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
	if ((position.length()>_limit) && (btDot(-position.normalized(), boid_front)< 0.7)) {//0.7
		dir = btVector3(-position.normalized()) ;
	}
}

btVector3 Boid::Avoid(std::vector <btRigidBody*> &obst) {
	btVector3 avoidVec = btVector3(0, 0, 0);
	btVector3 cylPos = btVector3(0, 0, 0);
	for (auto & cyl : obst) {

		//getting position with same y
		cylPos = btVector3(cyl->getWorldTransform().getOrigin());
		cylPos.setY(position.y());

		btScalar dist = btDistance(cylPos, position);
		//can see
		if (dist < 40) {//30
			if (btDot(boid_front, cylPos - position) > VISIBILITY) {//in front
				avoidVec = position - cylPos;
				DrawLine1(position, cylPos, colour.blue);
			}
		}
	}

	
	if(avoidVec.length()>1){
	avoidVec= avoidVec.safeNormalize();
	}
	body0->applyCentralForce(avoidVec*20);
	//body0->applyTorque(btVector3(0, avoidVec.getZ(), 0)*10);
	//body0->applyTorque(20.0 * boid_front.cross(avoidVec*10) - 10.0*avel);
	body0->applyTorque(20.0 * boid_front.cross(avoidVec * 2));
	return avoidVec;
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

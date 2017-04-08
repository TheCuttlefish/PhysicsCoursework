#include "Boid.h"
#include "GlutStuff.h"



using namespace std;


void Boid::Run(std::vector <Boid> &boids)
{
//-----> MAIN PHYSICS
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
		//apply torque/force
		body0->applyTorque(20.0 * boid_front.cross(Separation(boids) +
													Alignment(boids) +
													dir*PHYSICS_STRENGTH) - 10.0*avel);
		Cohesion(boids);//--applied only on force (not torque)

		body0->applyTorque(-6.5 * vec.up);//stay up   /-6.5
		body0->applyTorque(20.5 * boid_top.cross(vec.up) - 10 * avel);//left/right tilt
		LimitVelocity(MAX_VELOCITY);//apply velocity limit

		Wind();//wind can be toggled with button [2]

	}

	//limit the environment
	RadialLimit(MAX_DISTANCE);

	//draw axes
	//DrawLine1(position, position + boid_front * 15, colour.blue);
	//DrawLine1(position, position + boid_top * 15, colour.green);
	//DrawLine1(position, position + boid_right * 15, colour.red);


}

//-----> RULE 1: ALIGNMENT
btVector3 Boid::Alignment(std::vector <Boid> &boids) {
	btVector3 aVec = vec.zero;

	for (auto & boid : boids) {
		//can see
		if (boid.body0 != body0) {

			btScalar dist = btDistance(boid.position, position);//distance of vision
			if(dist<MAX_ALIGHNMENT_VISIBILITY){//50
				if (btDot(boid_front, boid.position - position)>VISIBILITY) {//in front
					aVec = aVec + btTransform(boid.body0->getOrientation())*vec.forward;
					//show who I'm looking at
					DrawLine1(position, position+(boid.position-position)/2, colour.black);
				}
			}
		}
	}

	CheckToNormalize(aVec);

		DrawLine1(position, position + aVec * 30, colour.green);
	return aVec*ALIGNMENT_STRENGTH;
}

//-----> RULE 2: COHESION
void Boid::Cohesion(std::vector <Boid> &boids) {
	btVector3 cVec = vec.zero;

	for (auto & boid : boids) {
		if (boid.body0 != body0) {

			btScalar dist = btDistance(boid.position, position);//distance of vision
			if (dist<MAX_COHESION_VISIBILITY && dist>MAX_SEPARATION_VISIBILITY) {//30
				if (btDot(boid_front, boid.position - position)>VISIBILITY) {//in front
					cVec = cVec + boid.position;
				}
			}
		}
	}
	CheckToNormalize(cVec);
	body0->applyCentralForce(cVec*COHESION_STRENGTH);
}


//-----> RULE 3: SEPARATION
btVector3 Boid::Separation(std::vector <Boid> &boids) {
	btVector3 sVec = vec.zero;

	for (auto & boid : boids) {
		//can see
		if (boid.body0 != body0) {

			btScalar dist = btDistance(boid.position, position);//distance of vision
			if (dist < MAX_SEPARATION_VISIBILITY) {//30
						if (btDot(boid_front, boid.position - position) > VISIBILITY) {//in front
							sVec = sVec + position - boid.position;
				}
			}
		}
	}

	CheckToNormalize(sVec);
	//sVec = sVec*-1;
	body0->applyCentralForce(sVec*SEPARATION_STRENGTH);
	return sVec*SEPARATION_STRENGTH;
}



//-----> RULE 3.1: AVOIDANCE
btVector3 Boid::Avoid(std::vector <btRigidBody*> &obst) {
	btVector3 avoidVec = vec.zero;
	btVector3 cylPos = vec.zero;
	for (auto & cyl : obst) {

		//getting position with the same y
		cylPos = btVector3(cyl->getWorldTransform().getOrigin());
		cylPos.setY(position.y());

		btScalar dist = btDistance(cylPos, position);//distance of vision
		if (dist < 40) {//30
			if (btDot(boid_front, cylPos - position) > VISIBILITY) {//in front
				avoidVec = position - cylPos;
				DrawLine1(position, cylPos, colour.blue);
			}
		}
	}

	CheckToNormalize(avoidVec);
	body0->applyCentralForce(avoidVec * 20);
	body0->applyTorque(20.0 * boid_front.cross(avoidVec * 2));
	return avoidVec;
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

void Boid::Wind() {
	if (windForce) {
		body0->applyCentralForce(btVector3(5, 0, 0));
	}
}


btVector3 Boid::CheckToNormalize(btVector3 &_vec) {
	 //normalise only if the vector's lenght is more than 1
	_vec.length() > 1 ? _vec = _vec.safeNormalize() : _vec;
	return _vec;
}


void Boid::DrawLine1(const btVector3 &_from, const btVector3 &_to, const btVector3 &_c) {

	if (!drawGizmos)return;

	glLineWidth(0.1f);
	glBegin(GL_LINES);
	glColor3f(_c.x(), _c.y(), _c.z());
	btglVertex3(_from.x(), _from.y(), _from.z());
	btglVertex3(_to.x(), _to.y(), _to.z());
	glEnd();
	
}



#include "Boid.h"
#include "GlutStuff.h"





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
		drag = -15 * vel;
		lift = -1.00 * gravity * vel.length();
		boid_front = trans * vec.forward;
		boid_top = trans * vec.up;
		boid_right = trans * vec.right;
		//add forces
		body0->applyTorque(10.0 * boid_front.cross(dir*1.1 + Alignment(boids) ) - 6.0*avel);
		//body0->applyTorque(10.0 * boid_front.cross(dir) - 6.0*avel);
		body0->applyTorque(-6.5 * vec.up);//stay up
		body0->applyTorque(10.5 * boid_top.cross(vec.up) - 10 * avel);//left/right tilt
		//LimitVelocity(MAX_VELOCITY);//apply velocity

		
		Cohesion(boids);

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

	DrawLine1(position, position + boid_front * 15, colour.blue);
	DrawLine1(position, position + boid_top * 15, colour.green);
	DrawLine1(position, position + boid_right * 15, colour.red);


}


btVector3 Boid::Alignment(std::vector <Boid> &boids) {
	btVector3 aVec = btVector3(0,0,0);
	
	for (auto & boid : boids) {
		//can see
		if (boid.body0 == body0) {

		} else {
			btScalar dist = btDistance(boid.position, position);
			//can see
			if(dist<20){//50
				if (btDot(boid_front, boid.position - position)<0) {//in front
					DrawLine1(position, boid.position, colour.green);
					aVec = aVec + btTransform(boid.body0->getOrientation())*vec.forward;
					aVec = aVec.safeNormalize();
				}
			}
			
		}

		
	}
	
	
	
	if (aVec.length() > .1) {
		DrawLine1(position, position + aVec * 30, colour.black);
	}
	else {
		DrawLine1(position, position + aVec * 30, colour.red);
	}

	return aVec*1;// *1
}

btVector3 Boid::Cohesion(std::vector <Boid> &boids) {
	btVector3 cVec = vec.zero;
	
	for (auto & boid : boids) {
		//can see
		if (boid.body0 == body0) {

		}
		else {
			btScalar dist = btDistance(boid.position, position);
			//can see
			if (dist<20) {//30
				if (btDot(boid_front, boid.position - position)>0) {//in front
					//Cohesion
					cVec = cVec + btTransform(boid.body0->getOrientation())*vec.forward;
					DrawLine1(position, position+cVec.safeNormalize()*50, colour.orange);
				
					if (dist < 15) {
						//cVec = -dir*10;
						//lift = lift*95;
						thrust = thrust * 0.95;
						
					}
				
				}
			}
		}


	}

	//get the average position
	
	cVec = cVec.safeNormalize();
	

	body0->applyTorque(cVec*1);//????5
	//10 is good
	return cVec;
}

btVector3 Boid::Seperation(std::vector <Boid> &boids) {
	btVector3 sVec = vec.zero;
	return sVec;
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
	if ((position.length()>_limit) && (btDot(-position.normalized(), boid_front)< 0.7)) {//0.5
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

	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(c.x(), c.y(), c.z());
	btglVertex3(from.x(), from.y(), from.z());
	btglVertex3(to.x(), to.y(), to.z());
	glEnd();

}


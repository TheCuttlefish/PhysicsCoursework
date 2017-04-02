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
		body0->applyTorque(10.0 * boid_front.cross(dir) - 6.0*avel);
		body0->applyTorque(-6.5 * vec.up);//stay up
		body0->applyTorque(10.5 * boid_top.cross(vec.up) - 10 * avel);//left/right tilt
		//LimitVelocity(MAX_VELOCITY);//apply velocity


		velocity = body0->getLinearVelocity().length();
		if (velocity < MAX_VELOCITY) {
			body0->applyCentralForce(thrust + lift + gravity + drag);
		}
		else {
			body0->applyCentralForce(lift + gravity + drag);
		}



	}

	
	RadialLimit(MAX_DISTANCE);


	DrawLine1(position, position + boid_front * 15, colour.blue);
	DrawLine1(position, position + boid_top * 15, colour.green);
	DrawLine1(position, position + boid_right * 15, colour.red);


}


btVector3 Boid::Alignment() {
	return vec.zero;
}

btVector3 Boid::Cohesion() {
	return vec.zero;
}

btVector3 Boid::Seperation() {
	return vec.zero;
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
	if ((position.length()>_limit) && (btDot(-position.normalized(), boid_front)< 0.5)) {
		dir = btVector3(-position.normalized());
		DrawLine1(position, vec.zero, colour.white);

	}
	else {
		dir = vec.zero;
		DrawLine1(position, vec.zero, colour.black);
	}
}




void Boid::DrawLine1(const btVector3 &from, const btVector3 &to, const btVector3 &c) {

	if (!drawGizmos)return;

	glLineWidth(2.0f);
	glBegin(GL_LINES);
	glColor3f(c.x(), c.y(), c.z());
	//glColor4f(1, 1, 0, .5); - for alpha
	btglVertex3(from.x(), from.y(), from.z());
	btglVertex3(to.x(), to.y(), to.z());
	glEnd();

}


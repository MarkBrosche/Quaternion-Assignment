/*
 * The Quat App, by Mark Brosche, was made by adapting the BigBallistic demo.  
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <gl/glew.h>	// Used by lighting system and drawing text on the window?
#include <gl/glut.h>
#include <cyclone.h>
#include <string>
#include "app.h"
#include "math.h"
#include "timing.h"
#include <stdio.h>
#include <iostream>
#include "utility.h"
#include <sstream>

using namespace std;

//Object names and misc.
double PI_2_val = 6.283185;

enum OBJECT_NAME
{
	OBJ_SUN,
	OBJ_SUN_MARK, 
	OBJ_EARTH, 
	OBJ_EARTH_MARK, 
	OBJ_MOON, 
	OBJ_MOON_MARK
};

int viewMode;
/** Vectors to hold camera information for gluLookAt. */
cyclone::Vector3 camera, focusPnt, upVector;

// object sizes, track sizes, and mark sizes: sun, earth (eth), moon (mun)
float	sunSize = 2.0f;			// radius of sun
float	sRefSize = sunSize / 8.0f;

float	ethOrbit = 16.0f;		// radius of earth orbit
float	ethSize = 1.0f;			// radius of earth
float	eRefSize = ethSize / 8.0f;

float	moonOrbit = 4.0f;		// radius of moon's orbit aroun earth
float	moonSize = 0.5f;	    // radius of moon
float	mRefSize = moonSize / 4.0f; 

//rotational periods and rotational velocity magnitude values
float	speedAdj = 0.020f;		// The update scale factor applied to every object's speed
float	sunRot = 27.275f;
float	ethRot = 1.0f;
float	ethOrbitPeriod = 365.25636f;
float	moonRot = 27.321660f;
float	moonOrbitPeriod = 27.321660f;
double	sunSpeed, 
		ethSpeed, 
		ethOrbitSpeed, 
		moonSpeed, 
		moonOrbitSpeed;

// Strings for identifying which rotational/orbital mode is in use
string  sun_r, earth_r, e_orbit_r, moon_r, m_orbit_r;

//sine and cosine values (used for constructing quaternions)
float sunCos, sunSin; 
float ethCos, ethSin; 
float ethOrbitCos, ethOrbitSin; 
float moonCos, moonSin; 
float moonOrbitCos, moonOrbitSin; 

//Object colors
cyclone::Vector3	sunColor = { 1.0f , 0.0f , 0.0f },
					earthClr = { 0.0f , 0.0f , 1.0f },
					moon_clr = { 1.0f , 1.0f , 0.0f },
					refColor = { 0.0f , 1.0f , 1.0f };
// Position and Rotation of the sun
cyclone::Vector3	sunCoords = { 0.0f, 0.0f, 0.0f },
					sunAxis,
					sunMarkStartPos,
					sunMarkPos;
cyclone::Quaternion	sunAccumQ,
					sunFrameQ;
// Position and Rotation of the earth
cyclone::Vector3	earthStartPos, 
					earthPos, 
					earthAxis,
					ethMarkStartPos, 
					ethMarkPos;
cyclone::Quaternion ethAccumQ, 
					ethFrameQ;
// Earth relative to the Sun
cyclone::Vector3	ethOrbitAxis;
cyclone::Quaternion ethOrbitAccumQ, 
					ethOrbitFrameQ;
// Position and Rotation of the moon
cyclone::Vector3	moonStartPos,
					moonPos, 
					moonAxis,
					moonMarkStartPos, 
					moonMarkPos;
cyclone::Quaternion moonAccumQ,
					moonFrameQ;
// Moon relative to Earth
cyclone::Vector3	moonOrbitAxisStart,
					moonOrbitAxis;
cyclone::Quaternion moonOrbitAccumQ, 
					moonOrbitFrameQ;

/** The CelestialBody class stores the information for instantiating
and updating spheres representing planetary bodies. */
class CelestialBody : public cyclone::CollisionSphere
{
public:
	OBJECT_NAME type;

	CelestialBody()
    {
        body = new cyclone::RigidBody;
    }

    ~CelestialBody()
    {
        delete body;
    }
	
	/** Draws the planet/sun/moon/mark, excluding its shadow. */
    void render(cyclone::Vector3 color)
    {     
		// Get the OpenGL transformation
		glColor3f(color.x, color.y, color.z);
		GLfloat mat[16];
		body->getGLTransform(mat);

		glPushMatrix();
		glMultMatrixf(mat);
		glutSolidSphere(radius , 20, 20);
		//glutSolidCube(radius);
		glPopMatrix();
    }

    /** Sets the planet initial state. */
    void setState(OBJECT_NAME objectName)
    {
		type = objectName;
		body->setOrientation(0.0f, 0.0f, 0.0f, 0.0f);
		body->setDamping(0.0f, 0.0f);
		// Set the properties of the body
		switch (type)
		{
		case (OBJ_SUN):
			body->setPosition(sunCoords.x, sunCoords.y, sunCoords.z);
			radius = sunSize;
			break;

		case (OBJ_SUN_MARK):
			body->setPosition(sunMarkStartPos.x, sunMarkStartPos.y, sunMarkStartPos.z);		
			radius = sRefSize;
			break;

		case (OBJ_EARTH):
			body->setPosition(earthStartPos.x, earthStartPos.y, earthStartPos.z);
			radius = ethSize;
			break;

		case (OBJ_EARTH_MARK):
			body->setPosition(ethMarkStartPos.x, ethMarkStartPos.y, ethMarkStartPos.z);
			radius = eRefSize;
			break;

		case (OBJ_MOON):
			body->setPosition(moonStartPos.x, moonStartPos.y, moonStartPos.z);
			radius = moonSize;
			break;

		case (OBJ_MOON_MARK):
			body->setPosition(moonMarkStartPos.x, moonMarkStartPos.y, moonMarkStartPos.z);
			radius = mRefSize;
			break;
		}

		body->setCanSleep(false);
		body->setAwake();
		body->setAcceleration(0.0f, 0.0f, 0.0f);
		//cyclone::Matrix3 tensor;
		//cyclone::real coeff = 0.4f*body->getMass()*radius*radius;
		//tensor.setInertiaTensorCoeffs(coeff, coeff, coeff);
		//body->setInertiaTensor(tensor);
		body->setLinearDamping(1.0f);
		body->setAngularDamping(1.0f);
		body->clearAccumulators();
		// Clear the force accumulators
		body->calculateDerivedData();
		calculateInternals();
    }
};

/** The main demo class definition. */
class QuatApp : public RigidBodyApplication
{
    /** Hold the planet rigidbody data. */
    CelestialBody sun[1], sunMark[1], earth[1], earthMark[1], moon[1], moon_Mark[1];

    /** Resets the position and orientation of bodies and camera. */
    virtual void reset();

    /** Build the contacts for the current situation. */
    virtual void generateContacts(); // will not use.

    /** Processes the objects in the simulation forward in time. */
    virtual void updateObjects(cyclone::real duration);

	/** Give objects velocity and spin amounts to update each frame. */
	void SetAngularVelocity(double * speed, 		
							float	 period,
							float  * selfCos,
							float  * selfSin,	
							cyclone::Vector3 &axis,	
							cyclone::Quaternion * frameQuat);

	/** Update an object based on its parent's position & rotation. */
	void UpdatePosition(OBJECT_NAME			  obj_name,
						cyclone::Quaternion * frameQuat,
						cyclone::Quaternion * accumulatedQuat,
						cyclone::Vector3	* parentObjPos,
						cyclone::Vector3	* startPos,		
						cyclone::Vector3	* newPos);

	/** Rotational modes.  Takes a char input representing the desired axis (x/y/z) about which to rotate. */
	void Sun_Rotation(unsigned char rotation);
	void Earth_Rotation(unsigned char rotation);
	void Moon_Rotation(unsigned char rotation);
	
	/** Orbital modes.  Takes a char input representing the desired axis (x/y/z) about which to rotate.  */
	void Moon_Orbit(unsigned char rotation);	
	
	/** Viewing modes. */
	void SunView();
	void EarthRadialView();
	void EarthTangentView();
	void EarthTopDownView();
	void MoonRadialView();
	void MoonTangentView();
	void MoonTopDownView();
				
public:
    /** Creates a new demo object. */
    QuatApp();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Sets up the rendering. */
    virtual void initGraphics();
    
    /** Display world. */
    virtual void display();

    /** Handle a keypress. */
    virtual void key(unsigned char key);
};

// Method definitions
QuatApp::QuatApp():RigidBodyApplication()
{
	pauseSimulation = false;
	// Reset all variables to their initial states
	reset();
}

void QuatApp::initGraphics()
{
	GLfloat lightAmbient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat lightDiffuse[] = { 0.5f, 0.35f, 0.6f, 1.0f };
	GLfloat lightSpecular[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	GLfloat lightPosition[] = { -1.0f, 1.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);
	glEnable(GL_COLOR_MATERIAL);
	
    Application::initGraphics();
}

void QuatApp::reset()
{
	// Reset the bodies to the default states and trajectories
	sunCoords = cyclone::Vector3(0.0f, 0.0f, 0.0f);
	sunAxis = cyclone::Vector3(0.0f, 0.0f, 1.0f);
	sunMarkStartPos = cyclone::Vector3(sunSize, 0.0f, 0.0f);
	sunMarkPos = sunMarkStartPos;
	sunAccumQ = cyclone::Quaternion();
	sunFrameQ = cyclone::Quaternion();
	sun->setState(OBJ_SUN);
	sun->body->setOrientation(0, 0, 0, 0);
	sunMark->setState(OBJ_SUN_MARK);
	sunMark->body->setOrientation(0, 0, 0, 0);
	sun_r = "Roll";

	earthStartPos = cyclone::Vector3(ethOrbit, 0.0f, 0.0f);
	earthPos = earthStartPos;
	earthAxis = cyclone::Vector3(1.0f, 0.0f, 0.0f);
	ethMarkStartPos = cyclone::Vector3(0.0f, 0.0f, ethSize);
	ethMarkPos = ethMarkStartPos;
	ethAccumQ = cyclone::Quaternion();
	ethFrameQ = cyclone::Quaternion();
	earth_r = "Pitch";

	ethOrbitAxis = cyclone::Vector3(0.0f, 1.0f, 0.0f);
	ethOrbitAccumQ = cyclone::Quaternion();
	ethOrbitFrameQ = cyclone::Quaternion();
	earth->setState(OBJ_EARTH);
	earth->body->setOrientation(0, 0, 0, 0);
	earthMark->setState(OBJ_EARTH_MARK);
	earthMark->body->setOrientation(0, 0, 0, 0);

	moonStartPos = cyclone::Vector3(moonOrbit, 0.0f, 0.0f);
	moonPos = moonStartPos;
	moonAxis = cyclone::Vector3(0.0f, 1.0f, 0.0f);
	moonMarkStartPos = cyclone::Vector3(moonSize, 0.0f, 0.0f);
	moonMarkPos = moonMarkStartPos;
	moonAccumQ = cyclone::Quaternion();
	moonFrameQ = cyclone::Quaternion();
	moon_r = "Yaw";

	moonOrbitAxisStart = cyclone::Vector3(0.0f, 0.0f, 1.0f);
	moonOrbitAxis = moonOrbitAxisStart;
	moonOrbitAccumQ = cyclone::Quaternion();
	moonOrbitFrameQ = cyclone::Quaternion();
	moon->setState(OBJ_MOON);
	moon->body->setOrientation(0, 0, 0, 0);
	moon_Mark->setState(OBJ_MOON_MARK);
	moon_Mark->body->setOrientation(0, 0, 0, 0);
	m_orbit_r = "Roll";

	// Look at the sun 
	viewMode = 1;

	// Start the objects moving!
	// Sun (sun reference mark)
	SetAngularVelocity( &sunSpeed, sunRot, &sunCos, &sunSin, sunAxis, &sunFrameQ);
	// Earth (earth reference mark)
	SetAngularVelocity( &ethSpeed, ethRot, &ethCos, &ethSin, earthAxis, &ethFrameQ);
	// Earth orbit
	SetAngularVelocity( &ethOrbitSpeed, ethOrbitPeriod, &ethOrbitCos, &ethOrbitSin, ethOrbitAxis, &ethOrbitFrameQ);
	// Moon (moon reference mark)
	SetAngularVelocity( &moonOrbitSpeed, moonRot, &moonCos, &moonSin, moonAxis, &moonFrameQ);
	// Moon orbit
	SetAngularVelocity( &moonSpeed, moonOrbitPeriod, &moonOrbitCos, &moonOrbitSin, moonOrbitAxis, &moonOrbitFrameQ);
}

const char* QuatApp::getTitle()
{
    return "Cyclone > Assignment 3: Quaternion App: Rotating Objects with Quaternions";
}

void QuatApp::SetAngularVelocity(double * speed,
								float	 period,
								float  * selfCos,
								float  * selfSin,
								cyclone::Vector3 &axis,
								cyclone::Quaternion * frameQuat)
{
	// Angular speed is calculated as the full rotation (2 pi) divided by the period it takes for a full rotation 
	// multiplied by the speed scale factor.
	*speed = -(PI_2_val * speedAdj / period);

	*selfCos = float(cos(*speed));
	*selfSin = float(sin(*speed));
	// The amount a quaternion accumulator is changed by every frame is the frame quat, and is based on the object's angular velocity:
	*frameQuat = cyclone::Quaternion((*selfCos), (*selfSin)*axis.x, (*selfSin)*axis.y, (*selfSin)*axis.z);
}

void QuatApp::UpdatePosition(	OBJECT_NAME	obj_name,	
								cyclone::Quaternion * frameQuat,	
								cyclone::Quaternion * accumulatedQuat,	
								cyclone::Vector3	* parentObjPos, 
								cyclone::Vector3	* startPos,		
								cyclone::Vector3	* newPos )
{
	// First combine the change in local rotation (frameQuat) with the quaternion accumulator and normalize to get the updated self-rotation Quat.
	cyclone::Quaternion newAccumQuat = *frameQuat;
	newAccumQuat *= *accumulatedQuat;
	*accumulatedQuat = newAccumQuat;
	accumulatedQuat->normalise();
	// The Transformation matrix M stores the quaternion's rotation matrix for transforming 
	// the previous position of the object into the new position 
	cyclone::Matrix4 matrixM; 

	if (obj_name == OBJ_SUN_MARK || obj_name == OBJ_EARTH)
	{
		matrixM.setOrientationAndPos(*accumulatedQuat, *parentObjPos); // Set up the matrix
		*newPos = matrixM.transform(*startPos); // local starting coords transformed into world coords
		if (obj_name = OBJ_SUN_MARK) 
		{
			sunMark->body->setPosition(*newPos);
			sunMark->body->setOrientation(newAccumQuat); 
			sun->body->setOrientation(newAccumQuat);
		}
		if (obj_name = OBJ_EARTH)
		{ 
			earth->body->setPosition(*newPos);
			earth->body->setOrientation(newAccumQuat); 
		}
	}
	// Now for bodies that are children of orbiting bodies, the self-rotation quat must combine with _all_ parent's orbits!
	if (obj_name == OBJ_EARTH_MARK || obj_name == OBJ_MOON)
	{
		newAccumQuat = (*accumulatedQuat);
		// Combine the accumulated quat with the earth orbit quat accumulation
		newAccumQuat *= ethOrbitAccumQ;
		newAccumQuat.normalise();
		matrixM.setOrientationAndPos(newAccumQuat, *parentObjPos); //parent object here is the earth
		*newPos = matrixM.transform(*startPos); // local starting coords transformed into world coords
		if (obj_name = OBJ_EARTH_MARK) 
		{ 
			earthMark->body->setPosition(*newPos); 
			earthMark->body->setOrientation(newAccumQuat); 
		}
		if (obj_name = OBJ_MOON) 
		{ 
			moon->body->setPosition(*newPos);
			moon->body->setOrientation(newAccumQuat);
		}
		// Don't forget to update the axis of rotation for the moon's orbit since it will change!
		moonOrbitAxis = matrixM.transformDirection(moonOrbitAxisStart);
	}
	if (obj_name == OBJ_MOON_MARK)
	{
		newAccumQuat = (*accumulatedQuat);
		// Combine the accumulated quat with the earth orbit moon orbit quat accumulations (!) in order (!)
		newAccumQuat *= moonOrbitAccumQ;
		newAccumQuat *= ethOrbitAccumQ;
		newAccumQuat.normalise();
		matrixM.setOrientationAndPos(newAccumQuat, *parentObjPos); //parent object here is the moon
		*newPos = matrixM.transform(*startPos); // local coordinates
		//apply new orientation and position to moon reference mark:
		moon_Mark->body->setPosition(*newPos);
		moon_Mark->body->setOrientation(newAccumQuat); 
	}
}

void QuatApp::updateObjects(cyclone::real duration)
{
    // Update the positions of the bodies.
	UpdatePosition(OBJ_SUN_MARK,	&sunFrameQ,			&sunAccumQ,			&sunCoords, &sunMarkStartPos,	&sunMarkPos);
	sunMark->calculateInternals();
	sunMark->body->integrate(duration);
	UpdatePosition(OBJ_EARTH,		&ethOrbitFrameQ,	&ethOrbitAccumQ,	&sunCoords, &earthStartPos,		&earthPos);
	earth->calculateInternals();
	earth->body->integrate(duration);
	UpdatePosition(OBJ_EARTH_MARK,	&ethFrameQ,			&ethAccumQ,			&earthPos,	&ethMarkStartPos,	&ethMarkPos);
	earthMark->calculateInternals();
	earthMark->body->integrate(duration);
	UpdatePosition(OBJ_MOON,		&moonOrbitFrameQ,	&moonOrbitAccumQ,	&earthPos,	&moonStartPos,		&moonPos);
	moon->calculateInternals();
	moon->body->integrate(duration);
	UpdatePosition(OBJ_MOON_MARK,	&moonFrameQ,		&moonAccumQ,		&moonPos,	&moonMarkStartPos,	&moonMarkPos);
	moon_Mark->calculateInternals();
	moon_Mark->body->integrate(duration);
}

void QuatApp::Sun_Rotation(unsigned char rotation)
{
	//Clear accumulator Quat
	sunAccumQ = cyclone::Quaternion();
	switch (rotation)
	{
	case 'x':
		//Set the new rotation axis
		sunAxis = cyclone::Vector3(1.0f, 0.0f, 0.0f);
		//Set the reference mark starting point such that it will revolve around the axis of rotation
		sunMarkStartPos = cyclone::Vector3(0.0f, 0.0f, sunSize);
		sun_r = "Pitch";
		break;
	case 'y':
		sunAxis = cyclone::Vector3(0.0f, 1.0f, 0.0f);
		sunMarkStartPos = cyclone::Vector3(0.0f, 0.0f, sunSize);
		sun_r = "Yaw";
		break;
	case 'z':
		sunAxis = cyclone::Vector3(0.0f, 0.0f, 1.0f);
		sunMarkStartPos = cyclone::Vector3(sunSize, 0.0f, 0.0f);
		sun_r = "Roll";
		break;
	}
	SetAngularVelocity( &sunSpeed, sunRot, &sunCos, &sunSin, sunAxis, &sunFrameQ);
}

void QuatApp::Earth_Rotation(unsigned char rotation)
{
	ethAccumQ = cyclone::Quaternion();
	switch (rotation)
	{
	case 'x':
		earthAxis = cyclone::Vector3(1.0f, 0.0f, 0.0f);
		ethMarkStartPos = cyclone::Vector3(0.0f, 0.0f, ethSize);
		earth_r = "Pitch";
		break;
	case 'y':
		earthAxis = cyclone::Vector3(0.0f, 1.0f, 0.0f);
		ethMarkStartPos = cyclone::Vector3(0.0f, 0.0f, ethSize);
		earth_r = "Yaw";
		break;
	case 'z':
		earthAxis = cyclone::Vector3(0.0f, 0.0f, 1.0f);
		ethMarkStartPos = cyclone::Vector3(ethSize, 0.0f, 0.0f);
		earth_r = "Roll";
		break;
	}
	SetAngularVelocity( &ethSpeed, ethRot, &ethCos, &ethSin, earthAxis, &ethFrameQ);
}

void QuatApp::Moon_Rotation(unsigned char rotation)
{
	moonAccumQ = cyclone::Quaternion();
	switch (rotation)
	{
	case 'x':
		moonAxis = cyclone::Vector3(1.0f, 0.0f, 0.0f);
		moonMarkStartPos = cyclone::Vector3(0.0f, 0.0f, moonSize);
		moon_r = "Pitch";
		break;
	case 'y':
		moonAxis = cyclone::Vector3(0.0f, 1.0f, 0.0f);
		moonMarkStartPos = cyclone::Vector3(0.0f, 0.0f, moonSize);
		moon_r = "Yaw";
		break;
	case 'z':
		moonAxis = cyclone::Vector3(0.0f, 0.0f, 1.0f);
		moonMarkStartPos = cyclone::Vector3(moonSize, 0.0f, 0.0f);
		moon_r = "Roll";
		break;
	}
	SetAngularVelocity( &moonSpeed, moonRot, &moonCos, &moonSin, moonAxis, &moonFrameQ);
}

void QuatApp::Moon_Orbit(unsigned char rotation)
{
	moonOrbitAccumQ = cyclone::Quaternion();
	//moonOrbitFrameQ = cyclone::Quaternion();

	switch (rotation)
	{
	case 'x':
		moonOrbitAxisStart = cyclone::Vector3(1.0f, 0.0f, 0.0f);
		moonStartPos = cyclone::Vector3(0.0f, 0.0f, moonOrbit);
		moonOrbitAxis = moonOrbitAxisStart;
		m_orbit_r = "Pitch";
		break;
	case 'y':
		moonOrbitAxisStart = cyclone::Vector3(0.0f, 1.0f, 0.0f);
		moonStartPos = cyclone::Vector3(0.0f, 0.0f, moonOrbit);
		moonOrbitAxis = moonOrbitAxisStart;
		m_orbit_r = "Yaw";
		break;
	case 'z':
		moonOrbitAxisStart = cyclone::Vector3(0.0f, 0.0f, 1.0f);
		moonStartPos = cyclone::Vector3(moonOrbit, 0.0f, 0.0f);
		moonOrbitAxis = moonOrbitAxisStart;
		m_orbit_r = "Roll";
		break;
	}
	SetAngularVelocity( &moonOrbitSpeed, moonOrbitPeriod, &moonOrbitCos, &moonOrbitSin, moonOrbitAxis, &moonOrbitFrameQ);
}

void QuatApp::display()
{
	// Clear the viewport and set the camera direction.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the bodies.
	sun->render(sunColor);
	sunMark->render(refColor);
	earth->render(earthClr);
	earthMark->render(refColor);
	moon->render(moon_clr);
	moon_Mark->render(refColor);

	// Compute the views and write out current view to the window.
	glColor3f(1.0, 1.0, 1.0);
	switch ( viewMode )
	{
	case 1: SunView();				renderText(10.0f, height - 24.0, "Sun View");				break;
	case 2: EarthRadialView();		renderText(10.0f, height - 24.0, "Earth Radial View");		break;
	case 3: EarthTangentView();		renderText(10.0f, height - 24.0, "Earth Tangent View");		break;
	case 4: EarthTopDownView();		renderText(10.0f, height - 24.0, "Earth Top-Down View");	break;
	case 5: MoonRadialView();		renderText(10.0f, height - 24.0, "Moon Radial View");		break;
	case 6: MoonTangentView();		renderText(10.0f, height - 24.0, "Moon Tangent View");		break;
	case 7: MoonTopDownView();		renderText(10.0f, height - 24.0, "Moon Top-Down View");		break;
	}
	// Write to the window the current rotational states of the bodies.
	renderText(10.0f, height - 48.0,  "Sun Rotation.....");
	printLargeString(sun_r);
	renderText(10.0f, height - 72.0,  "Earth Rotation.....");
	printLargeString(earth_r);
	renderText(10.0f, height - 96.0,  "Earth Orbit.....");
	printLargeString("Yaw");
	renderText(10.0f, height - 120.0, "Moon Rotation.....");
	printLargeString(moon_r);
	renderText(10.0f, height - 148.0, "Moon Orbit.....");
	printLargeString(m_orbit_r);

	// Set the Game Camera.
	glLoadIdentity();
	gluLookAt(
		camera.x,	camera.y,	camera.z,
		focusPnt.x, focusPnt.y, focusPnt.z,
		upVector.x, upVector.y, upVector.z
	);
}

void QuatApp::generateContacts()
{    
        // N/A - We aren't checking for collisions.
		// Create the ground plane data
	cyclone::CollisionPlane plane;
	plane.direction = cyclone::Vector3(0, 1, 0);
	plane.offset = -200.0f; // Collision plane lowered so the targets have a chance to fall down before removing

						  // Set up the collision data structure
	cData.reset(maxContacts);
	cData.friction = (cyclone::real)0.9;
	cData.restitution = (cyclone::real)0.1;
	cData.tolerance = (cyclone::real)0.01;
}

void QuatApp::key(unsigned char key)
{
    switch(key)
    {
	case 'r': case 'R': reset();		break; //or ???
	
	case 'q': case 'Q':		Sun_Rotation('y');		break;
	case 'w': case 'W':		Sun_Rotation('z');		break; //Default
	case 'e': case 'E':		Sun_Rotation('x');		break;
	case 'a': case 'A':		Earth_Rotation('y');	break;		
	case 's': case 'S':		Earth_Rotation('z');	break;
	case 'd': case 'D':		Earth_Rotation('x');	break; //Default
	case 'z': case 'Z':		Moon_Rotation('y');		break; //Default
	case 'x': case 'X':		Moon_Rotation('z');		break;
	case 'c': case 'C':		Moon_Rotation('x');		break;
	case 'b': case 'B':		Moon_Orbit('y');		break;
	case 'n': case 'N':		Moon_Orbit('z');		break; //Default
	case 'm': case 'M':		Moon_Orbit('x');		break;
    
	case '1':		viewMode = 1;		break;
	case '2':		viewMode = 2;		break;
	case '3':		viewMode = 3;		break;
	case '4':		viewMode = 4;		break;
	case '5':		viewMode = 5;		break;
	case '6':		viewMode = 6;		break;
	case '7':		viewMode = 7;		break;

	case 27:		exit(0);			break;
    }
 }

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new QuatApp();
}

void QuatApp::SunView() 
{
	camera = { 0.0f, 20.0f, 30.0f };
	focusPnt = sun->body->getPosition();
	upVector = { 0.0f, 1.0f, 0.0f };
}

void QuatApp::EarthRadialView() 
{
	float focusDist = ethSize * 20.0f;
	focusPnt = earthPos;
	cyclone::Vector3 sunDirection = earthPos;
	sunDirection.normalise();
	cyclone::Vector3 ethcam = sunDirection * focusDist;
	camera = { focusPnt.x + ethcam.x, focusPnt.y + ethcam.y, focusPnt.z + ethcam.z };
	upVector = { 0.0f, 1.0f, 0.0f };
}

void QuatApp::EarthTangentView() 
{
	float focusDist = ethSize * 18.0f;  // the distance between camera and subject
	upVector = { 0.0f, 1.0f, 0.0f };
	cyclone::Vector3 ethCam = earthPos % upVector; // perpindicular vector to find tangent
	ethCam.normalise();
	ethCam *= focusDist;
	focusPnt = earth->body->getPosition();
	camera = { focusPnt.x + ethCam.x, focusPnt.y + ethCam.y, focusPnt.z + ethCam.z };
}

void QuatApp::EarthTopDownView()
{
	float focusDist = ethSize * 18.0f;  // the distance between camera and subject
	cyclone::Vector3 up_vec = { 0.0f, 1.0f, 0.0f };
	cyclone::Vector3 tanVector = up_vec % earthPos; // perpindicular vector to find tangent
	cyclone::Vector3 ethCam = earthPos % tanVector; // perpindicular vector to find top of earth
	ethCam.normalise();
	ethCam *= focusDist;
	focusPnt = earth->body->getPosition();
	camera = { focusPnt.x + ethCam.x, focusPnt.y + ethCam.y, focusPnt.z + ethCam.z };
	upVector = { tanVector.x, tanVector.y, tanVector.z };
}

void QuatApp::MoonRadialView()
{
	float focusDist = moonSize * 10.0f; // the distance between camera and subject
	focusPnt = moon->body->getPosition();
	cyclone::Vector3 ethDirection = moonPos - earthPos;
	ethDirection.normalise();
	cyclone::Vector3 moonCam = ethDirection * focusDist;
	camera = { focusPnt.x + moonCam.x, focusPnt.y + moonCam.y, focusPnt.z + moonCam.z};
	upVector = moonOrbitAxis;
}

void QuatApp::MoonTangentView()
{
	float focusDist = moonSize * 10.0f; // the distance between camera and subject
	cyclone::Vector3 ethDirection = moonPos - earthPos;
	ethDirection.normalise();	
	cyclone::Vector3 moonCam = ethDirection % moonOrbitAxis; // perpindicular vector to find tangent
	moonCam.normalise();
	moonCam *= focusDist;
	focusPnt = moon->body->getPosition();
	camera = { focusPnt.x + moonCam.x, focusPnt.y + moonCam.y, focusPnt.z + moonCam.z };
	upVector = moonOrbitAxis;
}

void QuatApp::MoonTopDownView()
{
	float focusDist = moonSize * 10.0f; // the distance between camera and subject
	cyclone::Vector3 up_vec = { 0.0f, 1.0f, 0.0f };
	cyclone::Vector3 ethDirection = moonPos - earthPos;
	cyclone::Vector3 tanVector = moonOrbitAxis % ethDirection; // perpindicular vector
	tanVector.normalise();
	cyclone::Vector3 moonCam = moonOrbitAxis * focusDist;
	focusPnt = moon->body->getPosition();
	camera = { focusPnt.x + moonCam.x, focusPnt.y + moonCam.y, focusPnt.z + moonCam.z };
	upVector = { tanVector.x, tanVector.y, tanVector.z };
}


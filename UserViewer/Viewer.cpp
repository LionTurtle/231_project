/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "Viewer.h"

#if (ONI_PLATFORM == ONI_PLATFORM_MACOSX)
		#include <GLUT/glut.h>
#else
		#include <GL/glut.h>
#endif

#include <GL/glu.h>

#include <NiteSampleUtilities.h>
#include <math.h>

#define GL_WIN_SIZE_X	1280
#define GL_WIN_SIZE_Y	1024
#define TEXTURE_SIZE	512

#define DEFAULT_DISPLAY_MODE	DISPLAY_MODE_DEPTH

#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

SampleViewer* SampleViewer::ms_self = NULL;

bool g_drawSkeleton = true;
bool g_drawCenterOfMass = false;
bool g_drawStatusLabel = true;
bool g_drawBoundingBox = false;
bool g_drawBackground = true;
bool g_drawDepth = true;
bool g_drawFrameId = false;
bool g_drawHat = false;
bool g_drawCube = false;
bool g_drawCubeFront = false;

int g_nXRes = 0, g_nYRes = 0;

// time to hold in pose to exit program. In milliseconds.
const int g_poseTimeoutToExit = 2000;

void SampleViewer::glutIdle()
{
	glutPostRedisplay();
}
void SampleViewer::glutDisplay()
{
	SampleViewer::ms_self->Display();
}
void SampleViewer::glutKeyboard(unsigned char key, int x, int y)
{
	SampleViewer::ms_self->OnKey(key, x, y);
}

SampleViewer::SampleViewer(const char* strSampleName, openni::Device& device, openni::VideoStream& depth, openni::VideoStream& color) : m_poseUser(0), m_device(device), m_colorStream(color)
{
	ms_self = this;
	strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
	m_pUserTracker = new nite::UserTracker;
	m_eViewState = DISPLAY_MODE_DEPTH;
}
SampleViewer::~SampleViewer()
{
	Finalize();

	delete[] m_pTexMap;

	ms_self = NULL;
}

void SampleViewer::Finalize()
{
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status SampleViewer::Init(int argc, char **argv)
{
	m_pTexMap = NULL;

	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc-1; ++i)
	{
		if (strcmp(argv[i], "-device") == 0)
		{
			deviceUri = argv[i+1];
			break;
		}
	}

	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
	{
		return openni::STATUS_ERROR;
	}


	return InitOpenGL(argc, argv);

}
openni::Status SampleViewer::Run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char g_userStatusLabels[MAX_USERS][100] = {{0}};

char g_generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(g_userStatusLabels[user.getId()], "%s", msg);\
	printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

void updateUserState(const nite::UserData& user, uint64_t ts)
{
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
	}
	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{   
		glutBitmapCharacter(font,*str++);
	}   
}
#endif
void DrawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)g_nXRes;
	y *= GL_WIN_SIZE_Y/(float)g_nYRes;
	char *msg = g_userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}
void DrawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf(buffer, "%d", frameId);
	glColor3f(1.0f, 0.0f, 0.0f);
	glRasterPos2i(20, 20);
	glPrintString(GLUT_BITMAP_HELVETICA_18, buffer);
}
void DrawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[3] = {0};

	pUserTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	glPointSize(8);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

}
void DrawBoundingBox(const nite::UserData& user)
{
	glColor3f(1.0f, 1.0f, 1.0f);

	float coordinates[] =
	{
		user.getBoundingBox().max.x, user.getBoundingBox().max.y, 0,
		user.getBoundingBox().max.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().min.y, 0,
		user.getBoundingBox().min.x, user.getBoundingBox().max.y, 0,
	};
	coordinates[0]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[6]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[7]  *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[9]  *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[10] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINE_LOOP, 0, 4);

}



void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	float coordinates[6] = {0};
	pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;
	coordinates[3] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}
void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}

void DrawHat(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	const nite::SkeletonJoint& head = userData.getSkeleton().getJoint(nite::JOINT_HEAD);
	const nite::SkeletonJoint& neck = userData.getSkeleton().getJoint(nite::JOINT_NECK);
	const nite::Quaternion& headOrientation = head.getOrientation();

	float coordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &coordinates[0], &coordinates[1]);

	float neckCoordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(neck.getPosition().x, neck.getPosition().y, neck.getPosition().z, &neckCoordinates[0], &neckCoordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	neckCoordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	neckCoordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	float headOffset = 0.5*(coordinates[1]-neckCoordinates[1]);

	glPushMatrix();
	glTranslatef(coordinates[0], coordinates[1], 0);
	float pi = 3.14159;
	float theta = 2*pi-2*(float)acos(headOrientation.w);

	glRotatef(theta*180/pi, headOrientation.x/(float)sin(theta/2.f), headOrientation.y/(float)sin(theta/2.f), headOrientation.z/(float)sin(theta/2.f));

	glBegin(GL_TRIANGLES);
	glColor3f(0,0,1);
	glVertex2f(-headOffset/1.5, headOffset);
	glColor3f(1,0,0);
	glVertex2f(headOffset/1.5, headOffset);
	glColor3f(0,1,0);
	glVertex2f(0, 2.5*headOffset);
	glEnd();

	glPopMatrix();
}

void DrawCubeFront(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	const nite::SkeletonJoint& head = userData.getSkeleton().getJoint(nite::JOINT_HEAD);
	const nite::SkeletonJoint& neck = userData.getSkeleton().getJoint(nite::JOINT_NECK);
	const nite::Quaternion& headOrientation = head.getOrientation();

	const nite::SkeletonJoint& rShoulder = userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
	const nite::SkeletonJoint& lShoulder= userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);

	float diff = rShoulder.getPosition().z - (float)lShoulder.getPosition().z;

	float coordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &coordinates[0], &coordinates[1]);

	float neckCoordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(neck.getPosition().x, neck.getPosition().y, neck.getPosition().z, &neckCoordinates[0], &neckCoordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	neckCoordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	neckCoordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	float headOffset = 0.75*(coordinates[1]-neckCoordinates[1]);

	glPushMatrix();
	glTranslatef(coordinates[0], coordinates[1], 0);
	float pi = 3.14159;
	float theta;
	if (diff >= 0) {
		theta = 2*pi-2*(float)acos(headOrientation.w);
	} else {
		theta = 2*(float)acos(headOrientation.w);
	}
	
	// printf("%f\n", theta*180/pi);
	// printf("%f\n", headOrientation.w);

	glRotatef(theta*180/pi, headOrientation.x/(float)sin(theta/2.f), headOrientation.y/(float)sin(theta/2.f), headOrientation.z/(float)sin(theta/2.f));

	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f( headOffset,  2*headOffset, headOffset);
	glColor3f(0, 1, 0);
	glVertex3f(-headOffset,  2*headOffset, headOffset);
	glColor3f(0, 1, 1);
	glVertex3f(-headOffset, 0, headOffset);
	glColor3f(0, 0, 1);
	glVertex3f( headOffset, 0, headOffset);

	glEnd();  // End of drawing color-cube

	glPopMatrix();
}

void DrawCube(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	const nite::SkeletonJoint& head = userData.getSkeleton().getJoint(nite::JOINT_HEAD);
	const nite::SkeletonJoint& neck = userData.getSkeleton().getJoint(nite::JOINT_NECK);
	const nite::Quaternion& headOrientation = head.getOrientation();

	const nite::SkeletonJoint& rShoulder = userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
	const nite::SkeletonJoint& lShoulder= userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);

	float diff = rShoulder.getPosition().z - (float)lShoulder.getPosition().z;

	float coordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &coordinates[0], &coordinates[1]);

	float neckCoordinates[3] = {0};
	pUserTracker->convertJointCoordinatesToDepth(neck.getPosition().x, neck.getPosition().y, neck.getPosition().z, &neckCoordinates[0], &neckCoordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	neckCoordinates[0] *= GL_WIN_SIZE_X/(float)g_nXRes;
	neckCoordinates[1] *= GL_WIN_SIZE_Y/(float)g_nYRes;

	float headOffset = 0.75*(coordinates[1]-neckCoordinates[1]);

	glPushMatrix();
	glTranslatef(coordinates[0], coordinates[1], 0);
	float pi = 3.14159;
	float theta;
	if (diff >= 0) {
		theta = 2*pi-2*(float)acos(headOrientation.w);
	} else {
		theta = 2*(float)acos(headOrientation.w);
	}
	
	// printf("%f\n", theta*180/pi);
	// printf("%f\n", headOrientation.w);

	glRotatef(theta*180/pi, headOrientation.x/(float)sin(theta/2.f), headOrientation.y/(float)sin(theta/2.f), headOrientation.z/(float)sin(theta/2.f));

	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
	// Top face (y = 1.0f)
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	glColor3f(0.0f, 1.0f, 0.0f);     // Green
	glVertex3f( headOffset, 2*headOffset, -headOffset);
	glVertex3f(-headOffset, 2*headOffset, -headOffset);
	glVertex3f(-headOffset, 2*headOffset,  headOffset);
	glVertex3f( headOffset, 2*headOffset,  headOffset);

	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	glVertex3f( headOffset, 0,  headOffset);
	glVertex3f(-headOffset, 0,  headOffset);
	glVertex3f(-headOffset, 0, -headOffset);
	glVertex3f( headOffset, 0, -headOffset);

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f( headOffset,  2*headOffset, headOffset);
	glVertex3f(-headOffset,  2*headOffset, headOffset);
	glVertex3f(-headOffset, 0, headOffset);
	glVertex3f( headOffset, 0, headOffset);

	// Back face (z = -1.0f)
	glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	glVertex3f( headOffset, 0, -headOffset);
	glVertex3f(-headOffset, 0, -headOffset);
	glVertex3f(-headOffset,  2*headOffset, -headOffset);
	glVertex3f( headOffset,  2*headOffset, -headOffset);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(-headOffset,  2*headOffset,  headOffset);
	glVertex3f(-headOffset,  2*headOffset, -headOffset);
	glVertex3f(-headOffset, 0, -headOffset);
	glVertex3f(-headOffset, 0,  headOffset);

	// Right face (x = 1.0f)
	glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	glVertex3f(headOffset,  2*headOffset, -headOffset);
	glVertex3f(headOffset,  2*headOffset,  headOffset);
	glVertex3f(headOffset, 0,  headOffset);
	glVertex3f(headOffset, 0, -headOffset);
	glEnd();  // End of drawing color-cube

	glPopMatrix();
}

void SampleViewer::Display()
{
	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc != nite::STATUS_OK)
	{
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = userTrackerFrame.getDepthFrame();
	m_colorStream.readFrame(&m_colorFrame);

	if (m_pTexMap == NULL)
	{
		// Texture map init
		m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
	}

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -2000.0, 2000.0);

	if (depthFrame.isValid() && g_drawDepth)
	{
		calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
	}

	memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

	// check if we need to draw image frame to texture
	if (m_eViewState == DISPLAY_MODE_IMAGE && m_colorFrame.isValid())
	{
		const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + m_colorFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

		for (int y = 0; y < m_colorFrame.getHeight(); ++y)
		{
			const openni::RGB888Pixel* pImage = pImageRow;
			openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

			for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex)
			{
				*pTex = *pImage;
			}

			pImageRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	float factor[3] = {1, 1, 1};
	// check if we need to draw depth frame to texture
	if (depthFrame.isValid() && g_drawDepth)
	{
		const nite::UserId* pLabels = userLabels.getPixels();

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
			{
				if (*pDepth != 0)
				{
					if (*pLabels == 0)
					{
						if (!g_drawBackground)
						{
							factor[0] = factor[1] = factor[2] = 0;

						}
						else
						{
							factor[0] = Colors[colorCount][0];
							factor[1] = Colors[colorCount][1];
							factor[2] = Colors[colorCount][2];
						}
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					}
//					// Add debug lines - every 10cm
// 					else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

					int nHistValue = m_pDepthHist[*pDepth];
					pTex->r = nHistValue*factor[0];
					pTex->g = nHistValue*factor[1];
					pTex->b = nHistValue*factor[2];

					factor[0] = factor[1] = factor[2] = 1;
				}
			}

			pDepthRow += rowSize;
			pTexRow += m_nTexMapX;
		}
	}

	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nTexMapX, m_nTexMapY, 0, GL_RGB, GL_UNSIGNED_BYTE, m_pTexMap);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	g_nXRes = depthFrame.getVideoMode().getResolutionX();
	g_nYRes = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)g_nXRes/(float)m_nTexMapX, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)g_nYRes/(float)m_nTexMapY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];

		updateUserState(user, userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			m_pUserTracker->startSkeletonTracking(user.getId());
			m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
		}
		else if (!user.isLost())
		{
			if (g_drawStatusLabel)
			{
				DrawStatusLabel(m_pUserTracker, user);
			}
			if (g_drawCenterOfMass)
			{
				DrawCenterOfMass(m_pUserTracker, user);
			}
			if (g_drawBoundingBox)
			{
				DrawBoundingBox(user);
			}

			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawSkeleton)
			{
				DrawSkeleton(m_pUserTracker, user);
			}
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawHat)
			{
				DrawHat(m_pUserTracker, user);
			}
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawCube)
			{
				DrawCube(m_pUserTracker, user);
			}
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED && g_drawCubeFront)
			{
				DrawCubeFront(m_pUserTracker, user);
			}
		}

		if (m_poseUser == 0 || m_poseUser == user.getId())
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);

			if (pose.isEntered())
			{
				// Start timer
				sprintf(g_generalMessage, "In exit pose. Keep it for %d second%s to exit\n", g_poseTimeoutToExit/1000, g_poseTimeoutToExit/1000 == 1 ? "" : "s");
				printf("Counting down %d second to exit\n", g_poseTimeoutToExit/1000);
				m_poseUser = user.getId();
				m_poseTime = userTrackerFrame.getTimestamp();
			}
			else if (pose.isExited())
			{
				memset(g_generalMessage, 0, sizeof(g_generalMessage));
				printf("Count-down interrupted\n");
				m_poseTime = 0;
				m_poseUser = 0;
			}
			else if (pose.isHeld())
			{
				// tick
				if (userTrackerFrame.getTimestamp() - m_poseTime > g_poseTimeoutToExit * 1000)
				{
					printf("Count down complete. Exit...\n");
					Finalize();
					exit(2);
				}
			}
		}
	}

	if (g_drawFrameId)
	{
		DrawFrameId(userTrackerFrame.getFrameIndex());
	}

	if (g_generalMessage[0] != '\0')
	{
		char *msg = g_generalMessage;
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(100, 20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
	}

	// Swap the OpenGL display buffers
	glutSwapBuffers();

}

void SampleViewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
	case 's':
		// Draw skeleton?
		g_drawSkeleton = !g_drawSkeleton;
		break;
	case 'l':
		// Draw user status label?
		g_drawStatusLabel = !g_drawStatusLabel;
		break;
	case 'c':
		// Draw center of mass?
		g_drawCenterOfMass = !g_drawCenterOfMass;
		break;
	case 'x':
		// Draw bounding box?
		g_drawBoundingBox = !g_drawBoundingBox;
		break;
	case 'b':
		// Draw background?
		g_drawBackground = !g_drawBackground;
		break;
	case 'd':
		// Draw depth?
		g_drawDepth = !g_drawDepth;
		break;
	case 'f':
		// Draw frame ID
		g_drawFrameId = !g_drawFrameId;
		break;
	case 'h':
		// Draw hat
		g_drawHat = !g_drawHat;
		if (g_drawHat == true) {
			printf("Drawing hat!\n");
		}
		break;
	case 'k':
		// Draw cube
		g_drawCube = !g_drawCube;
		if (g_drawCube == true) {
			printf("Drawing cube!\n");
		}
		break;
	case 'q':
		// Draw cube
		g_drawCubeFront = !g_drawCubeFront;
		if (g_drawCubeFront == true) {
			printf("Drawing cube front!\n");
		}
		break;
	case 'i':
		// IMAGE MODE!
		if (m_eViewState == DISPLAY_MODE_IMAGE) {
			m_eViewState = DISPLAY_MODE_DEPTH;
		} else {
			m_eViewState = DISPLAY_MODE_IMAGE;
		}
		break;
	}
}

openni::Status SampleViewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (m_strSampleName);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;

}
void SampleViewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}

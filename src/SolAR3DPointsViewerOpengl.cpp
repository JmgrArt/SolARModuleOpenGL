/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolAR3DPointsViewerOpengl.h"
#include "core/Log.h"
#include <map>
#include <math.h>
#include <random>

namespace xpcf = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::OPENGL::SolAR3DPointsViewerOpengl)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace OPENGL {

static Transform3Df SolAR2GL = [] {
  Matrix<float, 4, 4> matrix;
  matrix << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  return Transform3Df(matrix);
}();

SolAR3DPointsViewerOpengl * SolAR3DPointsViewerOpengl::m_instance = NULL ;

SolAR3DPointsViewerOpengl::SolAR3DPointsViewerOpengl():ConfigurableBase(xpcf::toUUID<SolAR3DPointsViewerOpengl>())
{
    addInterface<api::display::I3DPointsViewer>(this);

    SRef<xpcf::IPropertyMap> params = getPropertyRootNode();

    params->wrapString("title",m_title);
    params->wrapUnsignedInteger("width", m_width);
    params->wrapUnsignedInteger("height", m_height);
    params->wrapUnsignedIntegerVector("backgroundColor", m_backgroundColor);
    params->wrapUnsignedInteger("fixedPointsColor", m_fixedPointsColor);
    params->wrapUnsignedIntegerVector("pointsColor", m_pointsColor);
    params->wrapUnsignedIntegerVector("points2Color", m_points2Color);
    params->wrapUnsignedIntegerVector("cameraColor", m_cameraColor);
    params->wrapUnsignedInteger("keyframeAsCamera", m_keyframeAsCamera);
    params->wrapUnsignedIntegerVector("framesColor", m_framesColor);
    params->wrapUnsignedIntegerVector("keyframesColor", m_keyframesColor);
    params->wrapUnsignedIntegerVector("keyframes2Color", m_keyframes2Color);
    params->wrapUnsignedInteger("drawCameraAxis", m_drawCameraAxis);
    params->wrapUnsignedInteger("drawSceneAxis", m_drawSceneAxis);
    params->wrapUnsignedInteger("drawWorldAxis", m_drawWorldAxis);
    params->wrapFloat("axisScale", m_axisScale);
    params->wrapFloat("pointSize", m_pointSize);
    params->wrapFloat("cameraScale", m_cameraScale);
    params->wrapFloat("zoomSensitivity", m_zoomSensitivity);
    params->wrapInteger("exitKey", m_exitKey);
    m_instance = this ;

   LOG_DEBUG(" SolAR3DPointsViewerOpengl constructor");

}

SolAR3DPointsViewerOpengl::~SolAR3DPointsViewerOpengl()
{
    LOG_DEBUG(" SolAR3DPointsViewerOpengl destructor")
}

xpcf::XPCFErrorCode SolAR3DPointsViewerOpengl::onConfigured()
{
    LOG_DEBUG(" SolAR3DPointsViewerOpengl onConfigured");
    char *myargv [1];
    int myargc=1;
    myargv [0]=strdup (m_title.c_str());
    glutInit(&myargc, myargv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    m_resolutionX = m_width;
    m_resolutionY = m_height;

    glutInitWindowSize(m_width, m_height);
    m_glWindowID = glutCreateWindow(m_title.c_str());
    glutDisplayFunc(Render);
    glutKeyboardFunc(KeyBoard);
    glutMouseFunc(MouseState);
    glutMotionFunc(MouseMotion);
    glutReshapeFunc(ResizeWindow);
    glutIdleFunc(MainLoop);
    glutMainLoopEvent();
    return xpcf::_SUCCESS;
}

FrameworkReturnCode SolAR3DPointsViewerOpengl::display (const std::vector<SRef<CloudPoint>>& points,
                                                        const Transform3Df & pose,
                                                        const std::vector<Transform3Df> keyframePoses,
                                                        const std::vector<Transform3Df> framePoses,
                                                        const std::vector<SRef<CloudPoint>>& points2,
                                                        const std::vector<Transform3Df> keyframePoses2)
{
    m_points = points;
    m_points2 = points2;
    m_cameraPose = pose;
    m_framePoses = framePoses;
    m_keyframePoses = keyframePoses;
    m_keyframePoses2 = keyframePoses2;

    if (m_firstDisplay)
    {
        // Compute the center point of the point cloud
        Point3Df minPoint, maxPoint;
        maxPoint(0)=(std::numeric_limits<size_t>::lowest)();
        maxPoint(1)=(std::numeric_limits<size_t>::lowest)();
        maxPoint(2)=(std::numeric_limits<size_t>::lowest)();
        minPoint(0)=(std::numeric_limits<size_t>::max)();
        minPoint(1)=(std::numeric_limits<size_t>::max)();
        minPoint(2)=(std::numeric_limits<size_t>::max)();

        for (int i = 0; i < m_points.size(); i++)
        {
            if (points[i]->getX() > maxPoint(0)) maxPoint(0)=points[i]->getX();
            if (points[i]->getY() > maxPoint(1)) maxPoint(1)=points[i]->getY();
            if (points[i]->getZ() > maxPoint(2)) maxPoint(2)=points[i]->getZ();
            if (points[i]->getX() < minPoint(0)) minPoint(0)=points[i]->getX();
            if (points[i]->getY() < minPoint(1)) minPoint(1)=points[i]->getY();
            if (points[i]->getZ() < minPoint(2)) minPoint(2)=points[i]->getZ();
        }
        Vector3f sceneDiagonal;

        // Center the scene on the center of the point cloud
        m_sceneCenter = Point3Df((minPoint(0)+maxPoint(0))/2.0f, -(minPoint(1)+maxPoint(1))/2.0f, -(minPoint(2)+maxPoint(2))/2.0f);

        // Add the camera to the box of the scene
        if (m_cameraPose(0,3) > maxPoint(0)) maxPoint(0)=m_cameraPose(0,3);
        if (m_cameraPose(1,3) > maxPoint(1)) maxPoint(1)=m_cameraPose(1,3);
        if (m_cameraPose(2,3) > maxPoint(2)) maxPoint(2)=m_cameraPose(2,3);
        if (m_cameraPose(0,3) < minPoint(0)) minPoint(0)=m_cameraPose(0,3);
        if (m_cameraPose(1,3) < minPoint(1)) minPoint(1)=m_cameraPose(1,3);
        if (m_cameraPose(2,3) < minPoint(2)) minPoint(2)=m_cameraPose(2,3);

        // Copmute the diagonal of the box to define the scene Size
        sceneDiagonal(0) = maxPoint(0) - minPoint(0);
        sceneDiagonal(1) = maxPoint(1) - minPoint(1);
        sceneDiagonal(2) = maxPoint(2) - minPoint(2);
        m_sceneSize = sceneDiagonal.norm();

        // Set the camera according to the center and the size of the scene.
        m_glcamera.resetview(math_vector_3f(m_sceneCenter.getX(), m_sceneCenter.getY(), m_sceneCenter.getY()), m_sceneSize);

        m_firstDisplay = false;
    }
    if (m_exitKeyPressed)
    {
        m_glcamera.clear(0.0, 0.0, 0.0, 1.0);
        glutDestroyWindow(m_glWindowID);
        return FrameworkReturnCode::_STOP;
    }

    glutMainLoopEvent();
    return FrameworkReturnCode::_SUCCESS;
}

void drawFrustumCamera(Transform3Df& pose,
                       std::vector<unsigned int>& color,
                       float scale,
                       float lineWidth,
                       bool displayCorner){

    // draw  camera pose !
    std::vector<Vector4f> cameraPyramid;
    Transform3Df glpose = SolAR2GL * pose;
    cameraPyramid.push_back(glpose * Vector4f(scale, scale, 2.0f*scale, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(-scale, scale, 2.0f*scale, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(-scale, -scale, 2.0f*scale, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(scale, -scale, 2.0f*scale, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(0, 0, 0, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(3.0f * scale, 0, 0, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(0, 3.0f * scale, 0, 1.0f));
    cameraPyramid.push_back(glpose * Vector4f(0, 0, 3.0f * scale, 1.0f));

    glColor3f(color[0], color[1], color[2]);
    if (displayCorner)
    {
        // draw a sphere at each corner of the frustum
        double cornerDiameter = 0.2f * scale;

        for (int i = 0; i < 5; ++i)
        {
         glPushMatrix();
         glTranslatef(cameraPyramid[i][0], cameraPyramid[i][1], cameraPyramid[i][2]);
         glutSolidSphere(cornerDiameter, 20, 20);
         glPopMatrix();
        }
    }

    // draw frustum lines
    glLineWidth(lineWidth);
    for (int i = 0; i < 4; ++i)
    {
     glBegin(GL_LINES);
     glVertex3f(cameraPyramid[4][0], cameraPyramid[4][1], cameraPyramid[4][2]);
     glVertex3f(cameraPyramid[i][0], cameraPyramid[i][1], cameraPyramid[i][2]);
     glEnd();
    }

    glBegin(GL_LINE_STRIP);
    glVertex3f(cameraPyramid[0][0], cameraPyramid[0][1], cameraPyramid[0][2]);
    glVertex3f(cameraPyramid[1][0], cameraPyramid[1][1], cameraPyramid[1][2]);
    glVertex3f(cameraPyramid[2][0], cameraPyramid[2][1], cameraPyramid[2][2]);
    glVertex3f(cameraPyramid[3][0], cameraPyramid[3][1], cameraPyramid[3][2]);
    glVertex3f(cameraPyramid[0][0], cameraPyramid[0][1], cameraPyramid[0][2]);
    glEnd();
}


void drawSphereCamera(Transform3Df& pose,
                      std::vector<unsigned int>& color,
                      float diameter){

    Transform3Df glPose = SolAR2GL * pose;
    GLUquadric * point = gluNewQuadric();
    glPushMatrix();
    glColor3f(color[0], color[1], color[2]);
    glTranslatef(glPose(0,3), glPose(1,3), glPose(2,3));
    gluSphere(point, GLdouble(diameter), GLint(20), GLint(20));
    glPopMatrix();


    gluDeleteQuadric(point);

}

void drawAxis(Transform3Df& pose, float scale, float lineWidth){

    Transform3Df glPose = SolAR2GL * pose;
    Vector4f center = glPose * Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    Vector4f x = glPose * Vector4f(scale, 0.0f, 0.0f, 1.0f);
    Vector4f y = glPose * Vector4f(0.0f, scale, 0.0f, 1.0f);
    Vector4f z = glPose * Vector4f(0.0f, 0.0f, scale, 1.0f);

    glLineWidth(lineWidth);
    // Draw x axis
    glColor3f(255, 0, 0);
    glBegin(GL_LINES);
    glVertex3f(center(0), center(1), center(2));
    glVertex3f(x(0), x(1), x(2));
    glEnd();
    // Draw y axis
    glColor3f(0, 255, 0);
    glBegin(GL_LINES);
    glVertex3f(center(0), center(1), center(2));
    glVertex3f(y(0), y(1), y(2));
    glEnd();
    // Draw z axis
    glColor3f(0, 0, 255);
    glBegin(GL_LINES);
    glVertex3f(center(0), center(1), center(2));
    glVertex3f(z(0), z(1), z(2));
    glEnd();
    glLineWidth(1.0f);
}



void SolAR3DPointsViewerOpengl::OnMainLoop()
{

}

void SolAR3DPointsViewerOpengl::OnRender()
{
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);

    m_glcamera.set_viewport(0, 0, m_resolutionX, m_resolutionY);
    m_glcamera.setup();
    m_glcamera.use_light(false);

    glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);

    if (m_drawWorldAxis)
    {
        Transform3Df identity = Transform3Df::Identity();
        drawAxis(identity, m_sceneSize * 0.1 * m_axisScale, m_axisScale);
    }

    if (m_drawSceneAxis)
    {
        Transform3Df sceneTransform = Transform3Df::Identity();
        sceneTransform(0,3)= m_sceneCenter[0];
        sceneTransform(1,3)= m_sceneCenter[1];
        sceneTransform(2,3)= m_sceneCenter[2];
        drawAxis(sceneTransform, m_sceneSize * 0.1 * m_axisScale, m_axisScale);
    }


    if(!m_points.empty())
    {
        glPushMatrix();
        glEnable (GL_POINT_SMOOTH);
        glPointSize(m_pointSize);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i < m_points.size(); ++i) {
         if (m_fixedPointsColor)
            glColor3f(m_pointsColor[0], m_pointsColor[1], m_pointsColor[2]);
         else
            glColor3f(m_points[i]->getR(), m_points[i]->getG(), m_points[i]->getB());

         glVertex3f(m_points[i]->getX(), -m_points[i]->getY(), -m_points[i]->getZ());
        }
        glEnd();
        glPopMatrix();
    }

    if(!m_points2.empty())
    {
        glPushMatrix();
        glEnable (GL_POINT_SMOOTH);
        glPointSize(m_pointSize);
        glBegin(GL_POINTS);
        for (unsigned int i = 0; i < m_points2.size(); ++i) {
         if (m_fixedPointsColor)
            glColor3f(m_points2Color[0], m_points2Color[1], m_points2Color[2]);
         else
            glColor3f(m_points2[i]->getR(), m_points2[i]->getG(), m_points2[i]->getB());

         glVertex3f(m_points2[i]->getX(), -m_points2[i]->getY(), -m_points2[i]->getZ());
        }
        glEnd();
        glPopMatrix();
    }

    // draw  camera pose !    
    std::vector<Vector4f> cameraPyramid;
    drawFrustumCamera(m_cameraPose, m_cameraColor, 0.033f * m_cameraScale * m_sceneSize, 0.003f * m_cameraScale * m_sceneSize, true);

    if (m_drawCameraAxis)
        drawAxis(m_cameraPose, m_sceneSize * 0.1 * m_axisScale, m_axisScale);

    // Draw keyframe poses
    if (!m_keyframePoses.empty())
    {
        glPushMatrix();
        if (m_keyframeAsCamera)
        {
            for (unsigned int i = 0; i < m_keyframePoses.size(); ++i)
                drawFrustumCamera(m_keyframePoses[i],m_keyframesColor, 0.013f * m_cameraScale * m_sceneSize,0.003f * m_cameraScale * m_sceneSize,false);
        }
        else
        {
            for (unsigned int i = 0; i < m_keyframePoses.size(); ++i)
                drawSphereCamera(m_keyframePoses[i], m_keyframesColor, 0.005f * m_cameraScale * m_sceneSize);
        }
        glPopMatrix();
    }

    // Draw keyframe poses for the second vector of keyframes
    if (!m_keyframePoses2.empty())
    {
        glPushMatrix();
        if (m_keyframeAsCamera)
        {
            for (unsigned int i = 0; i < m_keyframePoses2.size(); ++i)
                drawFrustumCamera(m_keyframePoses2[i],m_keyframes2Color, 0.013f * m_cameraScale * m_sceneSize,0.003f * m_cameraScale * m_sceneSize,false);
        }
        else
        {
            for (unsigned int i = 0; i < m_keyframePoses2.size(); ++i)
                drawSphereCamera(m_keyframePoses2[i], m_keyframes2Color, 0.005f * m_cameraScale * m_sceneSize);
        }
        glPopMatrix();
    }

    // Draw frame poses
    if (!m_framePoses.empty())
    {
        glPushMatrix();
        for (unsigned int i = 0; i < m_framePoses.size(); ++i)
            drawSphereCamera(m_framePoses[i], m_framesColor, 0.003f * m_cameraScale * m_sceneSize);
        glPopMatrix();
    }

    glLineWidth(1.0f);
    glutSwapBuffers();
    glutPostRedisplay();
}


void SolAR3DPointsViewerOpengl::OnResizeWindow(int _w, int _h)
{
    m_resolutionX = _w;
    m_resolutionY = _h;
}

void SolAR3DPointsViewerOpengl::OnKeyBoard(unsigned char key, int x, int y)
{
   if (key == m_exitKey)
       m_exitKeyPressed = true;
}


void SolAR3DPointsViewerOpengl::OnMouseMotion(int x, int y)
{
    y = m_resolutionY - y;
    m_glcamera.mouse_move(x, y);
}

void SolAR3DPointsViewerOpengl::OnMouseState(int button, int state, int x, int y)
{
    y = m_resolutionY - y;
    int zoom = 10;
    Mouse::button b = Mouse::NONE;

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        b = Mouse::ROTATE;
        m_glcamera.mouse(x, y, b);

    }
    else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {

        b = Mouse::MOVEXY;
        m_glcamera.mouse(x, y, b);
    }
    else if ((button & 3) == 3) {

        m_glcamera.mouse_wheel(m_zoomSensitivity);
    }
    else if ((button & 4) == 4) {

        m_glcamera.mouse_wheel(-m_zoomSensitivity);
    }
}

}
}
}

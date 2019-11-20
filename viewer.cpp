
#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

#include <osgGA/TerrainManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
//#include <osgGA/MatrixManipulator>

#include <osgManipulator/TabBoxDragger>
#include <osgManipulator/CommandManager>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/Point>
#include <osg/PositionAttitudeTransform>
#include <osg/LineWidth>

#include "BlockSparse.h"
#include "MathUtils.h"
#include "TetEqn.h"

#include <osgText/Text>

#include <sstream>
#include <set>
#include <vector>

#include <windows.h>

#include "ImplicitSolver.h"

ImplicitSolver *gSolver = NULL;
osg::ref_ptr<osg::Switch> ptsSwtc;
osg::ref_ptr<osg::Geometry> wireGeomBrick;
osg::ref_ptr<osg::Geometry> wireBrickPartial;
osg::ref_ptr<osg::Geometry> debugGeom;
osg::ref_ptr<osg::Geometry> debugGeom2;
osg::ref_ptr<osg::Geometry> debugGeom3;
osg::ref_ptr<osg::Geometry> debugGeom4;
osg::ref_ptr<osg::Geometry> debugGeom5;
std::vector<Math::Vector3> debugNodes(200000,Math::Vector3(0,0,0));
std::vector<Math::Vector3> debugNodes2(200000,Math::Vector3(0,0,0));
std::vector<Math::Vector3> debugNodes3(200000,Math::Vector3(0,0,0));
std::vector<Math::Vector3> debugNodes4(200000,Math::Vector3(0,0,0));
std::vector<Math::Vector3> debugNodes5(200000,Math::Vector3(0,0,0));
std::vector<Math::Vector3> partialNodes(200000,Math::Vector3(0,0,0));
std::vector<TetEqn<float>> g_render_tets;
std::vector<int> g_render_faultid;
std::vector<Math::Vector3> g_render_faultPoint(1000);
std::vector<Math::Vector3> g_render_faultNormal(1000);

static float ins = 0.f;
static float rot = 0.f;
static int iterChanged = 0;
static int run = 0;			// 0 = paused, 1 = running, 2 = single step
static int animate = 0;
static int showPoints = 0;



class PickHandler : public osgGA::GUIEventHandler {
public: 
	PickHandler() : _activeDragger(NULL) {}        
    ~PickHandler() {}    
    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
	{
		if ( gSolver )
		{
			if (ea.getEventType() == osgGA::GUIEventAdapter::EventType::KEYDOWN)
			{
				float multiplier = 1.f;
				bool relative = false;
				//if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT )
				//	multiplier = 0.03f;
				if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_SHIFT )
				{
					relative = true;
					multiplier = 0.75f;
				}
				if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL )
					multiplier = 0.25f;
				if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_RIGHT_CTRL )
					multiplier = 5.f;
				if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_RIGHT_SHIFT )
					multiplier = 30.f;


				if ( ea.getModKeyMask() == osgGA::GUIEventAdapter::MODKEY_LEFT_ALT )
				{
					if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left )
					{
					}
					if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right )
					{
					}
					if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up )
					{
					}
					if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Down )
					{
					}
					return false;
				}

			    if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Up )
			    {
				    ins += multiplier*0.4f;
				    iterChanged = 0;
			    }
 			    if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Down )
			    {
				    ins -= multiplier*0.4f;
				    iterChanged = 0;
			    }
			    if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Left )
			    {
				    rot += multiplier*1.7f;
				    iterChanged = 0;
			    }
 			    if ( ea.getKey() == osgGA::GUIEventAdapter::KEY_Right )
			    {
				    rot -= multiplier*1.7f;
				    iterChanged = 0;
			    }

				if ( ea.getKey() == 's' )
					run = 2;

				if ( ea.getKey() == 'r' )
				{
					run = run ? 0 : 1;

					if ( run ) 				
						gSolver->startThread();
					else
						gSolver->pauseThread();
				}

				if ( ea.getKey() == 'p' )
					showPoints = showPoints ? 0 : 1;
				ptsSwtc->setValue(0, 1 == showPoints );

				if ( ea.getKey() == 'a' )
					animate = animate ? 0 : 1;

			}

		}

		return false;
	}

private:
    osgManipulator::Dragger* _activeDragger;
    osgManipulator::PointerInfo _pointer;
};



class GroupCallback2 : public osg::NodeCallback
{
public:
	GroupCallback2()
	{
	}

	virtual void operator ()(osg::Node * node, osg::NodeVisitor * nv)
	{
		if ( gSolver )
		{
			if (run) {
				if (run == 2)
					run = 0;
			}

			++iterChanged;
            
			std::vector<Math::Vector3> brickNodes;
			std::vector<int> brickElems;
			gSolver->getNodes( 1, brickNodes, brickElems );
            
            int paccum = 0;
            for ( int r = 0; r < g_render_tets.size(); ++r ) {
                int *nodes = g_render_tets[r].getNodes();
                Math::Vector3 tetMidPt = 0.25*(brickNodes[nodes[0]] + brickNodes[nodes[1]] + brickNodes[nodes[2]] + brickNodes[nodes[3]]);
                int faultId = g_render_faultid[r];
                Math::Vector3 ptOnPlane = g_render_faultPoint[faultId];
                Math::Vector3 planeNormal = g_render_faultNormal[faultId];
                Math::Vector3 d3 = tetMidPt - ptOnPlane;
                float dn = Math::dot(d3,planeNormal );
                if ( dn < 0 )
                    continue;
                partialNodes[paccum++] = brickNodes[nodes[0]];
                partialNodes[paccum++] = brickNodes[nodes[1]];
                partialNodes[paccum++] = brickNodes[nodes[0]];
                partialNodes[paccum++] = brickNodes[nodes[2]];
                partialNodes[paccum++] = brickNodes[nodes[0]];
                partialNodes[paccum++] = brickNodes[nodes[3]];
                partialNodes[paccum++] = brickNodes[nodes[1]];
                partialNodes[paccum++] = brickNodes[nodes[2]];
                partialNodes[paccum++] = brickNodes[nodes[1]];
                partialNodes[paccum++] = brickNodes[nodes[3]];
                partialNodes[paccum++] = brickNodes[nodes[2]];
                partialNodes[paccum++] = brickNodes[nodes[3]];
            }

			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(brickNodes.size()); 
				for ( int i = 0; i < (int) brickNodes.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(brickNodes[i][0], brickNodes[i][1], brickNodes[i][2] );
                }
				wireGeomBrick->setVertexArray(vertices);
			}
            
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(debugNodes.size()); 
				for ( int i = 0; i < (int) debugNodes.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(debugNodes[i][0], debugNodes[i][1], debugNodes[i][2] );
                }
				//debugGeom->setVertexArray(vertices);
			}
            
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(debugNodes2.size()); 
				for ( int i = 0; i < (int) debugNodes2.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(debugNodes2[i][0], debugNodes2[i][1], debugNodes2[i][2] );
                }
				//debugGeom2->setVertexArray(vertices);
			}
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(debugNodes3.size()); 
				for ( int i = 0; i < (int) debugNodes3.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(debugNodes3[i][0], debugNodes3[i][1], debugNodes3[i][2] );
                }
				//debugGeom3->setVertexArray(vertices);
			}
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(debugNodes4.size()); 
				for ( int i = 0; i < (int) debugNodes4.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(debugNodes4[i][0], debugNodes4[i][1], debugNodes4[i][2] );
                }
				//debugGeom4->setVertexArray(vertices);
			}
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(debugNodes5.size()); 
				for ( int i = 0; i < (int) debugNodes5.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(debugNodes5[i][0], debugNodes5[i][1], debugNodes5[i][2] );
                }
				//debugGeom5->setVertexArray(vertices);
			}
            
			{
				osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(partialNodes.size()); 
				for ( int i = 0; i < (int) partialNodes.size(); ++i ) {
					(*vertices)[i] = osg::Vec3(partialNodes[i][0], partialNodes[i][1], partialNodes[i][2] );
                }
				//wireBrickPartial->setVertexArray(vertices);
			}
            


		}
		traverse (node, nv);
	}
};




int main( int argc, char **argv )
{

    osg::ArgumentParser arguments(&argc,argv);

	osg::ref_ptr<osg::Group> group = new osg::Group();


	osgViewer::Viewer viewer;
	viewer.setThreadingModel(osgViewer::ViewerBase::ThreadingModel::SingleThreaded);

	gSolver = new ImplicitSolver;
    gSolver->init();
	gSolver->pauseThread();
	

	{
		wireGeomBrick = new osg::Geometry();
		wireBrickPartial = new osg::Geometry();
		debugGeom = new osg::Geometry();
		debugGeom2 = new osg::Geometry();
		debugGeom3 = new osg::Geometry();
		debugGeom4 = new osg::Geometry();
		debugGeom5 = new osg::Geometry();

		osg::Vec4Array* colorsW = new osg::Vec4Array(1);  
		(*colorsW)[0] = osg::Vec4(1.f,1.f,1.f,1.f);  
		osg::Vec4Array* colorsWN = new osg::Vec4Array(1);  
		(*colorsWN)[0] = osg::Vec4(1.f,1.f,0.8f,1.f);  
		osg::Vec4Array* colorsG = new osg::Vec4Array(1);  
		(*colorsG)[0] = osg::Vec4(0.3f,1.f,0.f,1.f); 
		osg::Vec4Array* colorsGN = new osg::Vec4Array(1);  
		(*colorsGN)[0] = osg::Vec4(0.99f,0.7f,0.f,1.f);  
		osg::Vec4Array* colorsGB = new osg::Vec4Array(1);  
		(*colorsGB)[0] = osg::Vec4(0.99f,0.2f,0.f,9.f);  
        
		osg::Vec4Array* colorsG2 = new osg::Vec4Array(1);  
		(*colorsG2)[0] = osg::Vec4(0.3f,1.f,1.f,1.f); 
        
		osg::Vec4Array* colorsG3 = new osg::Vec4Array(1);  
		(*colorsG3)[0] = osg::Vec4(0.8f,0.2f,1.f,1.f); 
        
		osg::Vec4Array* colorsG4 = new osg::Vec4Array(1);  
		(*colorsG4)[0] = osg::Vec4(1.f,1.f,1.f,1.f); 
        
		wireGeomBrick->setColorArray(colorsW);  
		wireGeomBrick->setColorBinding(osg::Geometry::BIND_OVERALL); 		
		wireBrickPartial->setColorArray(colorsW);  
		wireBrickPartial->setColorBinding(osg::Geometry::BIND_OVERALL); 		
		debugGeom->setColorArray(colorsGB);  
		debugGeom->setColorBinding(osg::Geometry::BIND_OVERALL); 			
		debugGeom2->setColorArray(colorsG3);  
		debugGeom2->setColorBinding(osg::Geometry::BIND_OVERALL); 			
		debugGeom3->setColorArray(colorsG3);  
		debugGeom3->setColorBinding(osg::Geometry::BIND_OVERALL); 			
		debugGeom4->setColorArray(colorsG3);  
		debugGeom4->setColorBinding(osg::Geometry::BIND_OVERALL); 			
		debugGeom5->setColorArray(colorsW);  
		debugGeom5->setColorBinding(osg::Geometry::BIND_OVERALL); 		

		std::vector<int> brickElems;
		std::vector<Math::Vector3> brickNodes;
		gSolver->getNodes( 1, brickNodes, brickElems );
                
        int na = 6;
        int nb = 18;

		std::vector<int> debugElems;
        for ( int i = 0*na; i < 288*na; ++i ) {
            debugElems.push_back(i*2);
            debugElems.push_back(i*2+1);
        }

        std::vector<int> debugElems2;
        for ( int i = 0*nb; i < 288*nb; ++i ) {
            debugElems2.push_back(i*2);
            debugElems2.push_back(i*2+1);
        }
        
		wireGeomBrick->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,brickElems.size(),(GLuint*)&(brickElems.front())));
		wireBrickPartial->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems.size(),(GLuint*)&(debugElems.front())));
		debugGeom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems.size(),(GLuint*)&(debugElems.front())));
		debugGeom2->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems2.size(),(GLuint*)&(debugElems2.front())));
		debugGeom3->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems2.size(),(GLuint*)&(debugElems2.front())));
		debugGeom4->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems2.size(),(GLuint*)&(debugElems2.front())));
		debugGeom5->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::LINES,debugElems2.size(),(GLuint*)&(debugElems2.front())));

		osg::ref_ptr<osg::Geode> geode = new osg::Geode();
		group->addChild(geode);

		osg::ref_ptr<osg::Geode> geodeNodes = new osg::Geode();		
		ptsSwtc = new osg::Switch();
		group->addChild(ptsSwtc);
		ptsSwtc->addChild(geodeNodes);
		ptsSwtc->setValue(0,false);
        
		geode->addDrawable(wireGeomBrick); 	
		geode->addDrawable(wireBrickPartial);
		geode->addDrawable(debugGeom); 	
		geode->addDrawable(debugGeom2); 	
		geode->addDrawable(debugGeom3); 	
		geode->addDrawable(debugGeom4); 	
		geode->addDrawable(debugGeom5); 	

        osg::StateSet *stateset = geode->getOrCreateStateSet();
		osg::LineWidth* linewidth = new osg::LineWidth(); 
		linewidth->setWidth(1.0f); 
		stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON); 
		stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	}


	group->setUpdateCallback (new GroupCallback2);


	unsigned int windowWidth = 1024;
    unsigned int windowHeight = 1024;
    viewer.setUpViewInWindow(1000, 10, windowWidth, windowHeight);
	viewer.addEventHandler(new PickHandler);

	viewer.setSceneData(group.get());

	viewer.setCameraManipulator(new osgGA::TrackballManipulator());
	osg::Vec3d eye, center, up;
	viewer.getCameraManipulator()->getHomePosition(eye, center, up);
	eye = osg::Vec3f(eye[0], eye[2], eye[1]);
	up = osg::Vec3f(1, 0, 0);
	viewer.getCameraManipulator()->setHomePosition(eye, center, up);

	return viewer.run();
}



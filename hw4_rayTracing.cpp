#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include <iostream>
#include <fstream>
#include <cstring>

using namespace Ogre;
using namespace OgreBites;
using namespace std;


#define MAXDEPTH 3
#define MAXDISTANCE 2000.0

struct material_info {
    Vector3 Ka;
    Vector3 Ks;
    Vector3 Kd;
    int n;
    float reflect_ratio;
    float retract_ratio;
};

struct BVH_Node {
    AxisAlignedBox box;
    Entity** objects = NULL;
    BVH_Node* left_child = NULL;
    BVH_Node* right_child = NULL;
};

class BVH {
private:
    BVH_Node* root;
public:
    BVH() {
        root = new BVH_Node();

    }
    ~BVH() {
        delete root;
    }
    BVH_Node* getRoot() {
        return root;
    }
};

class RayTracing
    : public ApplicationContext, public InputListener
{
private:
    Root* root;
    SceneManager* scnMgr;
    Light* light;
    SceneNode* lightNode;
    SceneNode* camNode;
    Camera* cam;
    Entity* cube1;
    Entity* cube2;
    Entity* Mirror;
    Entity* plane;
    ManualObject* ground;
    ManualObject* mirror;
    SceneNode* ogreNode1;
    SceneNode* ogreNode2;
    SceneNode* ogreNode3;
    SceneNode* ogreNode4;
    RaySceneQuery* mRaySceneQuery;
    Vector3 light_pos;
    Vector3 camera_pos;
    material_info cube;
    material_info barrel;
    material_info mirror_m;
    material_info land;
    AxisAlignedBox root_box;
    AxisAlignedBox box1;
    AxisAlignedBox box2;
public:
    RayTracing();
    virtual ~RayTracing() {}
    void setup();
    void imageGenerate();
    void rayTrace(Ray, bool, int, Vector3&);
    bool keyPressed(const KeyboardEvent& evt);
    Entity** BVH_Accelerate(Ray);
    void getMeshInformation(const MeshPtr mesh, size_t& vertex_count, Vector3*& vertices, size_t& index_count, unsigned long*& indices, const Vector3& position, const Quaternion& orient, const Vector3& scale);
};

Vector3 current_ray_origin;

RayTracing::RayTracing()
    : ApplicationContext("MyFirstApp")
{
}


void RayTracing::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();

    // get a pointer to the already created root
    root = getRoot();
    scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    //! [turnlights]
    scnMgr->setAmbientLight(ColourValue(0.15, 0.15, 0.15));
    //! [turnlights]
    scnMgr->setShadowTechnique(ShadowTechnique::SHADOWTYPE_STENCIL_ADDITIVE);

    //! [newlight]
    light = scnMgr->createLight("MainLight");
    lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    //! [newlight]

    //! [lightpos]
    lightNode->setPosition(-100, 500, 200);
    //! [lightpos]

    //! [camera]
    camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

    // create the camera
    cam = scnMgr->createCamera("myCam");
    //cam->setNearClipDistance(5); 
    cam->setDirection(Vector3(0.5, -1, -2));
    cam->setAutoAspectRatio(true);
    //cam->setCastShadows(true);
    camNode->attachObject(cam);
    camNode->setPosition(0, 0, 140);

    // and tell it to render into the main window
    getRenderWindow()->addViewport(cam);
    //! [camera]

    
    cube1= scnMgr->createEntity("cube","cube.mesh");
    //cube1->setMaterialName("Examples/RustySteel");
    cube1->setCastShadows(true);
    ogreNode1 = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode1->attachObject(cube1);
    ogreNode1->setPosition(0, 0, 0);

    cube2 = scnMgr->createEntity("barrel","cube.mesh");
    //cube2->setMaterialName("Examples/Hilite/Yellow");
    cube2->setCastShadows(true);
    ogreNode2 = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode2->attachObject(cube2);
    ogreNode2->setPosition(150, 0, 0);
    //ogreNode2->setScale(10, 10, 10);

    mirror = scnMgr->createManualObject("mirror");
    mirror->begin("white", RenderOperation::OT_TRIANGLE_LIST);
    mirror->position(300, -50, -200);
    mirror->normal(-1, 0, 1);
    mirror->textureCoord(0, 0);

    mirror->position(500, -50, 0);
    mirror->normal(-1, 0, 1);
    mirror->textureCoord(1, 0);

    mirror->position(500, 150, 0);
    mirror->normal(-1, 0, 1);
    mirror->textureCoord(1, 1);

    mirror->position(300, 150, -200);
    mirror->normal(-1, 0, 1);
    mirror->textureCoord(0, 1);

    mirror->quad(0, 1, 2, 3);
    mirror->end();
    MeshPtr mp = mirror->convertToMesh("Mirror");

    Mirror = scnMgr->createEntity("mirror",mp);
    //Mirror->setMaterialName("Examples/glass");
    Mirror->setCastShadows(true);
    ogreNode3 = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode3->attachObject(Mirror);
    ogreNode3->setPosition(0, 0, 0);
    //ogreNode3->setScale(0.7, 0.7, 0.7);

    ground = scnMgr->createManualObject("ground");
    ground->begin("Ogre/Tusks", RenderOperation::OT_TRIANGLE_LIST);
    ground->position(-150, -50, 200);
    ground->normal(0, 0, 1);
    ground->textureCoord(0, 0);

    ground->position(500, -50, 200);
    ground->normal(0, 0, 1);
    ground->textureCoord(1, 0);

    ground->position(500, -50, -200);
    ground->normal(0, 0, 1);
    ground->textureCoord(1, 1);

    ground->position(-150, -50, -200);
    ground->normal(0, 0, 1);
    ground->textureCoord(0, 1);

    ground->quad(0, 1, 2, 3);
    ground->end();
    MeshPtr mp1 = ground->convertToMesh("Ground");

    plane = scnMgr->createEntity("land",mp1);
    plane->setCastShadows(false);
    ogreNode4 = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode4->attachObject(plane);
    ogreNode4->setPosition(0, 0, 0);

    light_pos = lightNode->getPosition();
    camera_pos = camNode->getPosition();

    camNode->setPosition(0, 250, 500);
    //imageGenerate();
    cube.Ka = Vector3(0.3, 0.3, 0.3);
    cube.Kd = Vector3(0.7, 0.7, 0.2);
    cube.Ks = Vector3(0.2, 0.2, 0);
    cube.n = 1;
    cube.reflect_ratio = 0;
    cube.retract_ratio = 0;

    barrel.Ka = Vector3(0.3, 0.3, 0.3);
    barrel.Kd = Vector3(0.7, 0.4, 0.5);
    barrel.Ks = Vector3(1, 1, 1);
    barrel.n = 1;
    barrel.reflect_ratio = 0.2;
    barrel.retract_ratio = 0;

    mirror_m.Ka = Vector3(0.2, 0.2, 0.2);
    mirror_m.Kd = Vector3(0.7, 0.7, 1);
    mirror_m.Ks = Vector3(1, 1, 1);
    mirror_m.n = 1;
    mirror_m.reflect_ratio = 1;
    mirror_m.retract_ratio = 0;

    land.Ka = Vector3(0.3, 0.3, 0.3);
    land.Kd = Vector3(0.1, 0.5, 0.1);
    land.Ks = Vector3(0.1, 0.1, 0.1);
    land.n = 0;
    land.reflect_ratio = 0;
    land.retract_ratio = 0;

    
}

void RayTracing::imageGenerate()
{
    ofstream outImage;
    outImage.open("D:/rayTracing.ppm");
    if (!outImage) {
        cout << "fail to open file" << endl;
        return;
    }
    auto w = getRenderWindow();
    int height = w->getHeight();
    int width = w->getWidth();
    mRaySceneQuery = scnMgr->createRayQuery(Ray());
    outImage << "P3\n" << width << ' ' << height << "\n255\n";
    int depth;
    float ratio;
    Vector3 color;
    for (int i = 1; i <= height; i++) {
        float screeny = i / float(height);
        for (int j = 1; j <= width; j++) {
            float screenx = j / float(width);
            Ray r = cam->getCameraToViewportRay(screenx, screeny);
            /*mRaySceneQuery->setRay(r);
            RaySceneQueryResult& result = mRaySceneQuery->execute();
            cout << result.size() << endl;*/
            /*Raytestresult result = r.intersects(s);
            if (result.first) {
                cout << result.second << endl;
            }*/
            depth = 0;
            current_ray_origin = r.getOrigin();
            rayTrace(r, false, depth, color);
            int cr = int(255.99 * color.x);
            int cg = int(255.99 * color.y);
            int cb = int(255.99 * color.z);
            outImage << cr << ' ' << cg << ' ' << cb << '\n';
        }
    }
    cout << "success!" << endl;
    outImage.close();
    scnMgr->destroyQuery(mRaySceneQuery);
}

void RayTracing::rayTrace(Ray R, bool reflecting, int depth, Vector3& color)
{ 
    if (depth > MAXDEPTH) {
        color = Vector3(0, 0, 0);
        return;
    }
    mRaySceneQuery->setRay(R);
    mRaySceneQuery->setQueryTypeMask(SceneManager::ENTITY_TYPE_MASK);
    mRaySceneQuery->setSortByDistance(true);
    //mRaySceneQuery->setWorldFragmentType(SceneQuery::WorldFragmentType::WFT_RENDER_OPERATION);
    RaySceneQueryResult& result = mRaySceneQuery->execute();
    //cout << result.size() << endl;
    if (mRaySceneQuery->execute().size() <= 0) {
        color = Vector3(0, 0, 0);
        return;
    }
    //cout << mRaySceneQuery->execute().size() << endl;
    Real closest_distance = MAXDISTANCE;
    RaySceneQueryResult& query_result = mRaySceneQuery->getLastResults();
    //if (query_result.size() > 2) cout << query_result.size()<<' ';
    for (size_t idx = 0; idx < query_result.size(); idx++) {
        //cout << query_result[idx].movable->getMovableType() << endl;
        if ((closest_distance >= 0.0f) && (closest_distance < query_result[idx].distance)) {
            break;
        }
        if ((query_result[idx].movable != NULL) && (query_result[idx].movable->getMovableType().compare("Entity") == 0)) {
            Entity* e = static_cast<Entity*>(query_result[idx].movable);
            Vector3 intersect_point = R.getPoint(query_result[idx].distance);
            Vector3 L = light_pos - intersect_point;
            Vector3 V = -R.getDirection();
            Ray shadow_ray = Ray(intersect_point, L);
            mRaySceneQuery->setRay(shadow_ray);
            string entity_name = e->getName();
            //if (query_result.size() > 2) cout << entity_name << endl;
            material_info mi;
      
            if (entity_name == "cube") {
                mi = cube;
            }
            else if (entity_name == "mirror") {
                mi = mirror_m;
            }
            else if (entity_name == "barrel") {
                mi = barrel;
            }
            else if (entity_name == "land") {
                mi = land;
            }
     
            if ((mRaySceneQuery->execute().size() > 0)&&(!reflecting)) {
                color = Vector3::ZERO;
            }
            else {
                L.normalise();
                V.normalise();
                
                size_t vertex_count;
                size_t index_count;
                Vector3* vertices;
                unsigned long* indices;
                getMeshInformation(e->getMesh(), vertex_count, vertices, index_count, indices, e->getParentNode()->_getDerivedPosition(), e->getParentNode()->_getDerivedOrientation(), e->getParentNode()->_getDerivedScale());
                if (index_count % 3 != 0) {
                    index_count -= index_count % 3;
                }
                bool new_closest_found = false;
                int hit_index = 0;
                for (int i = 0; i < static_cast<int>(index_count); i += 3) {
                    std::pair<bool, Real> hit = Math::intersects(R, vertices[indices[i]], vertices[indices[i + 1]], vertices[indices[i + 2]], true, true);
                    if (hit.first) {
                        if ((closest_distance < 0.0f) || (hit.second < closest_distance)) {
                            closest_distance = hit.second;
                            new_closest_found = true;
                            hit_index = i;
                        }
                    }
                }
                Vector3 N = Math::calculateBasicFaceNormal(vertices[indices[hit_index]], vertices[indices[hit_index + 1]], vertices[indices[hit_index + 2]]);
                N.normalise();
                Vector3 R, R1, Is, Id, Ia, local_color, reflect_color, transmitted_color;
                Vector3 lightN = Vector3(0, -1, 0);
                Vector3 H = L + V;
                H.normalise();
                R = 2 * L.absDotProduct(N) * N - L;
                R1 = 2 * V.absDotProduct(N) * N - V;
                Id = mi.Kd * ((N.dotProduct(L) > 0) ? N.dotProduct(L) : 0) * ((lightN.dotProduct(-L) > 0) ? lightN.dotProduct(-L) : 0);
                Is = mi.Ks * Math::Pow(((N.dotProduct(H) > 0) ? N.dotProduct(H) : 0), mi.n)* ((lightN.dotProduct(-L) > 0) ? lightN.dotProduct(-L) : 0);
                Ia = mi.Ka;
                local_color = Ia + Id + Is;
                reflect_color = transmitted_color = Vector3::ZERO;
                Ray reflect_ray = Ray(intersect_point, R1);
                rayTrace(reflect_ray, true, depth + 1, reflect_color);

                /*if (entity_name == "barrel") {
                    Ray transmit_ray = Ray(intersect_point,-V);
                    rayTrace(transmit_ray, true, depth + 1, transmitted_color);
                }*/
                color = local_color + (mi.reflect_ratio) * reflect_color + (mi.retract_ratio) * transmitted_color;
            }
            return;
        }
    }

}

bool RayTracing::keyPressed(const KeyboardEvent& evt)
{
    if (evt.keysym.sym == SDLK_ESCAPE)
    {
        imageGenerate();
    }
    return false;
}

Entity** RayTracing::BVH_Accelerate(Ray r)
{
    //BVH and bounding boxes initialize:
    box1 = cube1->getBoundingBox();
    box1.merge(cube2->getBoundingBox());
    box2 = Mirror->getBoundingBox();
    box2.merge(plane->getBoundingBox());
    root_box = box1;
    root_box.merge(box2);

    BVH bvh;
    BVH_Node* bvh_root = bvh.getRoot();
    bvh_root->box = root_box;
    bvh_root->objects = NULL;
    bvh_root->left_child = new BVH_Node();
    bvh_root->right_child = new BVH_Node();
    BVH_Node* left = bvh_root->left_child;
    BVH_Node* right = bvh_root->right_child;
    left->box = box1;
    right->box = box2;
    left->objects = new Entity * [2];
    right->objects = new Entity * [2];
    left->objects[0] = cube1;
    left->objects[1] = cube2;
    right->objects[0] = Mirror;
    right->objects[1] = plane;
    //begin to test intersections:
    RayTestResult rtr = r.intersects(bvh_root->box);
    if (!rtr.first) {
        return NULL;
    }
    else {
        RayTestResult rtr1 = r.intersects(left->box);
        if (rtr1.first) {
            return left->objects;
        }
        else {
            RayTestResult rtr2 = r.intersects(right->box);
            if (rtr2.first) {
                return right->objects;
            }
            else {
                return NULL;
            }
        }
    }
    
}

void RayTracing::getMeshInformation(const MeshPtr mesh, size_t& vertex_count, Vector3*& vertices, size_t& index_count, unsigned long*& indices, const Vector3& position, const Quaternion& orient, const Vector3& scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;
    vertex_count = index_count = 0;
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i) {
        SubMesh* submesh = mesh->getSubMesh(i);
        if (submesh->useSharedVertices) {
            if (!added_shared) {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else {
            vertex_count += submesh->vertexData->vertexCount;
        }
        index_count += submesh->indexData->indexCount;
    }
    vertices = new Vector3[vertex_count];
    indices = new unsigned long[index_count];
    added_shared = false;
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i) {
        SubMesh* submesh = mesh->getSubMesh(i);
        VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared)) {
            if (submesh->useSharedVertices) {
                added_shared = true;
                shared_offset = current_offset;
            }
            const VertexElement* posElem = vertex_data->vertexDeclaration->findElementBySemantic(VES_POSITION);
            HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(HardwareBuffer::HBL_READ_ONLY));
            float* pReal;
            for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize()) {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Vector3 pt(pReal[0], pReal[1], pReal[2]);
                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }
            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }
        IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
        bool use32bitindexes = (ibuf->getType() == HardwareIndexBuffer::IT_32BIT);
        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);
        size_t offset = (submesh->useSharedVertices) ? shared_offset : current_offset;
        if (use32bitindexes) {
            for (size_t k = 0; k < numTris * 3; ++k) {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }
        }
        else {
            for (size_t k = 0; k < numTris * 3; ++k) {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) + static_cast<unsigned long>(offset);
            }
        }
        ibuf->unlock();
        current_offset = next_offset;
    }
}

int main(int argc, char** argv)
{
    try
    {
        RayTracing app;
        app.initApp();
        app.getRoot()->startRendering();
        app.imageGenerate();
        app.closeApp();
    }
    catch (const exception & e)
    {
        cerr << "Error occurred during execution: " << e.what() << '\n';
        return 1;
    }
    return 0;
}


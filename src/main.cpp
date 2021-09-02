#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "raylib.h"
#include "raymath.h"
#include <cstdio>

void setTransform(btScalar m[16], Matrix *matrix) {
  matrix->m0 = m[0];
  matrix->m1 = m[1];
  matrix->m2 = m[2];
  matrix->m3 = m[3];
  matrix->m4 = m[4];
  matrix->m5 = m[5];
  matrix->m6 = m[6];
  matrix->m7 = m[7];
  matrix->m8 = m[8];
  matrix->m9 = m[9];
  matrix->m10 = m[10];
  matrix->m11 = m[11];
  matrix->m12 = m[12];
  matrix->m13 = m[13];
  matrix->m14 = m[14];
  matrix->m15 = m[15];
}

void AllocateMeshData(Mesh *mesh, int triangleCount) {
  mesh->vertexCount = triangleCount * 3;
  mesh->triangleCount = triangleCount;

  mesh->vertices = (float *)MemAlloc(mesh->vertexCount * 3 * sizeof(float));
  mesh->texcoords = (float *)MemAlloc(mesh->vertexCount * 2 * sizeof(float));
  mesh->normals = (float *)MemAlloc(mesh->vertexCount * 3 * sizeof(float));
}

Mesh ShapeToMesh(btCollisionShape *shape) {
  Mesh mesh = {0};

  if (shape->isConvex()) {

    const btConvexPolyhedron *poly =
        shape->isPolyhedral()
            ? ((btPolyhedralConvexShape *)shape)->getConvexPolyhedron()
            : 0;

    if (poly) {
      int i;
      AllocateMeshData(&mesh, poly->m_faces.size());
      int currentVertice = 0;
      for (i = 0; i < poly->m_faces.size(); i++) {
        btVector3 centroid(0, 0, 0);
        int numVerts = poly->m_faces[i].m_indices.size();
        if (numVerts > 2) {
          btVector3 v1 = poly->m_vertices[poly->m_faces[i].m_indices[0]];
          for (int v = 0; v < poly->m_faces[i].m_indices.size() - 2; v++) {
            btVector3 v2 = poly->m_vertices[poly->m_faces[i].m_indices[v + 1]];
            btVector3 v3 = poly->m_vertices[poly->m_faces[i].m_indices[v + 2]];
            btVector3 normal = (v3 - v1).cross(v2 - v1);
            normal.normalize();

            mesh.vertices[currentVertice] = v1.x();
            mesh.vertices[currentVertice + 1] = v1.y();
            mesh.vertices[currentVertice + 2] = v1.z();
            mesh.normals[currentVertice] = normal.getX();
            mesh.normals[currentVertice + 1] = normal.getY();
            mesh.normals[currentVertice + 2] = normal.getZ();

            mesh.vertices[currentVertice + 3] = v2.x();
            mesh.vertices[currentVertice + 4] = v2.y();
            mesh.vertices[currentVertice + 5] = v2.z();
            mesh.normals[currentVertice + 3] = normal.getX();
            mesh.normals[currentVertice + 4] = normal.getY();
            mesh.normals[currentVertice + 5] = normal.getZ();

            mesh.vertices[currentVertice + 6] = v3.x();
            mesh.vertices[currentVertice + 7] = v3.y();
            mesh.vertices[currentVertice + 8] = v3.z();
            mesh.normals[currentVertice + 6] = normal.getX();
            mesh.normals[currentVertice + 7] = normal.getY();
            mesh.normals[currentVertice + 8] = normal.getZ();

            currentVertice += 9;
          }
        }
      }
    } else {
      btConvexShape *convexShape = (btConvexShape *)shape;
      btShapeHull *hull = new btShapeHull(convexShape);
      hull->buildHull(shape->getMargin());

      AllocateMeshData(&mesh, hull->numTriangles());
      int currentVertice = 0;
      if (hull->numTriangles() > 0) {

        int index = 0;
        const unsigned int *idx = hull->getIndexPointer();
        const btVector3 *vtx = hull->getVertexPointer();

        for (int i = 0; i < hull->numTriangles(); i++) {
          int i1 = index++;
          int i2 = index++;
          int i3 = index++;
          btAssert(i1 < hull->numIndices() && i2 < hull->numIndices() &&
                   i3 < hull->numIndices());

          int index1 = idx[i1];
          int index2 = idx[i2];
          int index3 = idx[i3];
          btAssert(index1 < hull->numVertices() &&
                   index2 < hull->numVertices() &&
                   index3 < hull->numVertices());

          btVector3 v1 = vtx[index1];
          btVector3 v2 = vtx[index2];
          btVector3 v3 = vtx[index3];
          btVector3 normal = (v3 - v1).cross(v2 - v1);
          normal.normalize();

          mesh.vertices[currentVertice] = v1.x();
          mesh.vertices[currentVertice + 1] = v1.y();
          mesh.vertices[currentVertice + 2] = v1.z();
          mesh.normals[currentVertice] = normal.getX();
          mesh.normals[currentVertice + 1] = normal.getY();
          mesh.normals[currentVertice + 2] = normal.getZ();

          mesh.vertices[currentVertice + 3] = v2.x();
          mesh.vertices[currentVertice + 4] = v2.y();
          mesh.vertices[currentVertice + 5] = v2.z();
          mesh.normals[currentVertice + 3] = normal.getX();
          mesh.normals[currentVertice + 4] = normal.getY();
          mesh.normals[currentVertice + 5] = normal.getZ();

          mesh.vertices[currentVertice + 6] = v3.x();
          mesh.vertices[currentVertice + 7] = v3.y();
          mesh.vertices[currentVertice + 8] = v3.z();
          mesh.normals[currentVertice + 6] = normal.getX();
          mesh.normals[currentVertice + 7] = normal.getY();
          mesh.normals[currentVertice + 8] = normal.getZ();

          currentVertice += 9;
        }
      }
    }
    UploadMesh(&mesh, false);
  }
  return mesh;
}

struct PhysicsWorld {
  // keep the collision shapes, for deletion/cleanup
  btAlignedObjectArray<btCollisionShape *> m_collisionShapes;
  btAlignedObjectArray<Model> m_models;
  btBroadphaseInterface *m_broadphase;
  btCollisionDispatcher *m_dispatcher;
  btConstraintSolver *m_solver;
  btDefaultCollisionConfiguration *m_collisionConfiguration;
  btDiscreteDynamicsWorld *m_dynamicsWorld;

  PhysicsWorld()
      : m_broadphase(0), m_dispatcher(0), m_solver(0),
        m_collisionConfiguration(0), m_dynamicsWorld(0) {}

  virtual ~PhysicsWorld() {}

  btDiscreteDynamicsWorld *getDynamicsWorld() { return m_dynamicsWorld; }

  virtual void initPhysics() {
    /// collision configuration contains default setup for memory, collision
    /// setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    /// use the default collision dispatcher. For parallel processing you can
    /// use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();

    /// the default constraint solver. For parallel processing you can use a
    /// different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver *sol =
        new btSequentialImpulseConstraintSolver;
    m_solver = sol;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(
        m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

    m_dynamicsWorld->setGravity(btVector3(0, -10, 0));
  }

  virtual void exitPhysics() {
    // cleanup in the reverse order of creation/initialization
    // remove the rigidbodies from the dynamics world and delete them

    if (m_dynamicsWorld) {
      int i;
      for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--) {
        m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
      }
      for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject *obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody *body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
          delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;
      }
    }
    // delete collision shapes
    for (int j = 0; j < m_collisionShapes.size(); j++) {
      btCollisionShape *shape = m_collisionShapes[j];
      delete shape;
    }
    m_collisionShapes.clear();

    // delete raylib's models
    for (int j = 0; j < m_models.size(); j++) {
      UnloadModel(m_models[j]);
    }
    m_models.clear();

    delete m_dynamicsWorld;
    m_dynamicsWorld = 0;

    delete m_solver;
    m_solver = 0;

    delete m_broadphase;
    m_broadphase = 0;

    delete m_dispatcher;
    m_dispatcher = 0;

    delete m_collisionConfiguration;
    m_collisionConfiguration = 0;
  }

  btRigidBody *createRigidBody(float mass, const btTransform &startTransform,
                               btCollisionShape *shape, int modelIndex,
                               btVector3 colorRGB = btVector3(130, 130, 130)) {

    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      shape->calculateLocalInertia(mass, localInertia);

    // using motionstate is recommended, it provides interpolation
    // capabilities, and only synchronizes 'active' objects
    btDefaultMotionState *myMotionState =
        new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
                                                   localInertia);

    btRigidBody *body = new btRigidBody(cInfo);

    body->setUserIndex(-1);
    body->setUserIndex3(modelIndex);

    body->setCustomDebugColor(colorRGB);

    m_dynamicsWorld->addRigidBody(body);
    return body;
  }

  void deleteRigidBody(btRigidBody *body) {
    int graphicsUid = body->getUserIndex();
    m_dynamicsWorld->removeRigidBody(body);
    btMotionState *ms = body->getMotionState();
    delete body;
    delete ms;
  }

  void drawDebug() {
    for (int j = m_dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--) {
      btCollisionObject *obj = m_dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody *body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState()) {
        body->getMotionState()->getWorldTransform(trans);
      } else {
        trans = obj->getWorldTransform();
      }

      btScalar m[16];
      trans.getOpenGLMatrix(m);
      Vector3 position = (Vector3){0, 0, 0};
      Model model = this->m_models[body->getUserIndex3()];
      setTransform(m, &model.transform);

      btVector3 vecColor = btVector3(230, 41, 55);
      body->getCustomDebugColor(vecColor);
      Color color = (Color){(unsigned char)vecColor.getX(),
                            (unsigned char)vecColor.getY(),
                            (unsigned char)vecColor.getZ(), 255};

      DrawModelWires(model, position, 1.0f, color);
    }
  }
};

int main() {
  const int screenWidth = 1024;
  const int screenHeight = 720;
  InitWindow(screenWidth, screenHeight, "raylib - bullet3");
  Camera camera = {{2.0f, 0.1f, 2.0f},
                   {3.0f, -2.0f, 3.0f},
                   {0.0f, 1.0f, 0.0f},
                   45.0f,
                   CAMERA_PERSPECTIVE};
  SetCameraMode(camera, CAMERA_FIRST_PERSON); // Set camera mode

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second

  // physics
  //--------------------------------------------------------------------------------------
  PhysicsWorld world = *new PhysicsWorld();
  world.initPhysics();

  {
    btCollisionShape *groundShape =
        new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
    world.m_collisionShapes.push_back(groundShape);

    Model model = LoadModelFromMesh(ShapeToMesh(groundShape));
    world.m_models.push_back(model);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));

    btRigidBody *ground = world.createRigidBody(
        0., groundTransform, groundShape, world.m_models.size() - 1);
  }

  {
    btCollisionShape *colShape = new btSphereShape(btScalar(1.));
    world.m_collisionShapes.push_back(colShape);

    Model model = LoadModelFromMesh(ShapeToMesh(colShape));
    world.m_models.push_back(model);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(2, 10, 0));

    btRigidBody *sphere = world.createRigidBody(1.f, startTransform, colShape,
                                                world.m_models.size() - 1,
                                                btVector3(255, 161, 0));
  }

  {
    // create a few dynamic rigidbodies
    // Re-using the same collision is better for memory usage and performance

    btBoxShape *colShape =
        new btBoxShape(btVector3(btScalar(.5), btScalar(.5), btScalar(.5)));

    // btCollisionShape* colShape = new btSphereShape(btScalar(1.));
    world.m_collisionShapes.push_back(colShape);

    Model model = LoadModelFromMesh(ShapeToMesh(colShape));
    world.m_models.push_back(model);

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    btScalar mass(1.f);

    // rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      colShape->calculateLocalInertia(mass, localInertia);

    int arraySizeY = 5;
    int arraySizeX = 5;
    int arraySizeZ = 5;

    for (int k = 0; k < arraySizeY; k++) {
      for (int i = 0; i < arraySizeX; i++) {
        for (int j = 0; j < arraySizeZ; j++) {
          startTransform.setOrigin(btVector3(
              btScalar(0.5 * i), btScalar(5 + .5 * k), btScalar(0.5 * j)));

          world.createRigidBody(mass, startTransform, colShape,
                                world.m_models.size() - 1,
                                btVector3(0, 121, 241));
        }
      }
    }
  }

  btCollisionShape *colShapeBullet = new btSphereShape(btScalar(.25));
  world.m_collisionShapes.push_back(colShapeBullet);

  Model modelBullet = LoadModelFromMesh(ShapeToMesh(colShapeBullet));
  world.m_models.push_back(modelBullet);

  int bulletShapeIndex = world.m_models.size() - 1;

  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    world.m_dynamicsWorld->stepSimulation(GetFrameTime(), 10);
    UpdateCamera(&camera); // Update camera

    BeginDrawing();
    {
      ClearBackground(RAYWHITE);

      BeginMode3D(camera);
      {
        world.drawDebug();

        Vector3 pos = camera.position;
        Vector3 target = camera.target;
        Vector3 direction = Vector3Normalize(Vector3Subtract(target,pos));

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
          btTransform startTransform;
          startTransform.setIdentity();
          startTransform.setOrigin(btVector3(target.x, target.y, target.z));

          btRigidBody *sphere =
              world.createRigidBody(1.f, startTransform, colShapeBullet,
                                    bulletShapeIndex, btVector3(255, 50, 50));

          sphere->applyImpulse(btVector3(direction.x*50,direction.y*50,direction.z*50), btVector3(1,0,1));
        }
      }
      EndMode3D();

      DrawFPS(10, 10);
    }
    EndDrawing();
  }

  world.exitPhysics();
  CloseWindow();

  return 0;
}

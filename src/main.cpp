#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btBulletDynamicsCommon.h"
#include "raylib-cpp.hpp"
#include "raylib.h"

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

struct PhysicsWorld {
  // keep the collision shapes, for deletion/cleanup
  btAlignedObjectArray<btCollisionShape *> m_collisionShapes;
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
                               btCollisionShape *shape) {
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

  void draw(Model box, Model ball) {
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
      Vector3 position = (Vector3){0,0,0};

      int shapeType = body->getCollisionShape()->getShapeType();
      if (shapeType == BOX_SHAPE_PROXYTYPE) {
        setTransform(m, &box.transform);
        DrawModelWires(box, position, 1.0f, RED);

      } else if (shapeType == SPHERE_SHAPE_PROXYTYPE) {
        setTransform(m, &ball.transform);
        DrawModelWires(ball, position, 1.0f, BLUE);
      }

      printf("world pos object %d = %f,%f,%f\n", j,
             float(trans.getOrigin().getX()), float(trans.getOrigin().getY()),
             float(trans.getOrigin().getZ()));
    }
  }
};

int main() {
  const int screenWidth = 800;
  const int screenHeight = 450;
  raylib::Window window(screenWidth, screenHeight, "raylib");
  raylib::Camera3D camera(
    raylib::Vector3(0.0f, 10.0f, 10.0f),
    raylib::Vector3(0.0f, -5.0f, 0.0f),
    raylib::Vector3(0.0f, 10.0f, 0.0f),
    45.0f, CAMERA_PERSPECTIVE);
  camera.SetMode(CAMERA_ORBITAL); // Set an orbital camera mode

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second

  // raylib models
  Model box = LoadModelFromMesh(GenMeshCube(100., 100., 100.));
  Model ball = LoadModelFromMesh(GenMeshSphere(1., 32, 32));

  // physics
  //--------------------------------------------------------------------------------------
  PhysicsWorld world = *new PhysicsWorld();
  world.initPhysics();

  {
    btCollisionShape *groundShape =
        new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
    world.m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -56, 0));

    btRigidBody *ground =
        world.createRigidBody(0., groundTransform, groundShape);
  }

  {
    btCollisionShape *colShape = new btSphereShape(btScalar(1.));
    world.m_collisionShapes.push_back(colShape);

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(2, 10, 0));

    btRigidBody *ground = world.createRigidBody(1.f, startTransform, colShape);
  }

  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!window.ShouldClose()) { // Detect window close button or ESC key
    // Update
    //----------------------------------------------------------------------------------
    world.m_dynamicsWorld->stepSimulation(GetFrameTime(), 10);
    camera.Update(); // Update camera
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();
    {
      window.ClearBackground(RAYWHITE);

      camera.BeginMode();
      {
        world.draw(box, ball);
      }
      camera.EndMode();

      DrawFPS(10, 10);
    }
    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  world.exitPhysics();
  return 0;
}

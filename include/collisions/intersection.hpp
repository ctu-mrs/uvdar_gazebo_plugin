#include <gazebo/physics/ode/ODEPhysics.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include "collisions/collision_std.h"
#include "collisions/collision_trimesh_internal.h"
#include "collisions/heightfield.h"
#include "collisions/collision_kernel.h"
#include "collisions/odemath.h"
#include <chrono>

#define INIFINITE_RANGE 1000

//dMaxUserClasses = 4
//these are the defined tyeps of geometry:
/* enum { */
/* 00   dSphereClass = 0, */
/* 01   dBoxClass, */
/* 02   dCapsuleClass, */
/* 03   dCylinderClass, */
/* 04   dPlaneClass, */
/* 05   dRayClass, */
/* 06   dConvexClass, */
/* 07   dGeomTransformClass, */
/* 08   dTriMeshClass, */
/* 09   dHeightfieldClass, */

/* 10   dFirstSpaceClass, */
/* 10   dSimpleSpaceClass = dFirstSpaceClass, */
/* 11   dHashSpaceClass, */
/* 12   dSweepAndPruneSpaceClass, // SAP */
/* 13   dQuadTreeSpaceClass, */
/* 13   dLastSpaceClass = dQuadTreeSpaceClass, */

/* 14   dFirstUserClass, */
/* 17   dLastUserClass = dFirstUserClass + dMaxUserClasses - 1, */
/* 18   dGeomNumClasses */
/* }; */


#define IS_SPACE(geom) \
  ((geom)->type >= dFirstSpaceClass && (geom)->type <= dLastSpaceClass)

#define GEOM_ENABLED(g) (((g)->gflags & GEOM_ENABLE_TEST_MASK) == GEOM_ENABLE_TEST_VALUE)

namespace ODERayHack {
  class Intersection
  {
    /// \brief Depth of the ray intersection.
    public: double depth;

            /// \brief Name of the collision object that was hit.
    public: std::string name;
  };

  class RayIntersectorHack
  {

    private:
      bool debug = false;
      dxSpace* world_space;
      dxGeom* world_geom;
      gazebo::physics::ODERayShapePtr ode_ray;
      double led_distance = INIFINITE_RANGE;
      bool got_obstacle = false;


      int curr_depth_indent = 0;
      std::string camera_name;
      std::vector<std::pair<std::string,int>>  elapsedTime;

    public:
      RayIntersectorHack(
          gazebo::physics::PhysicsEnginePtr pengine,
          std::string camera_name_i,
          bool debug_i = false
          ){
        debug = debug_i;
        camera_name = camera_name_i;

        world_space = (boost::static_pointer_cast<gazebo::physics::ODEPhysics>(pengine))->GetSpaceId();
        world_geom = (dGeomID)(world_space);
        if (!world_geom){
          std::cerr << "[UV CAM collisions]: ["<< camera_name <<"]: Could not access space for occlusion retrieval. Returning." << std::endl;
          return;
        }

        ode_ray = boost::dynamic_pointer_cast<gazebo::physics::ODERayShape>(
            pengine->CreateShape("ray", gazebo::physics::CollisionPtr()));

      }

    private:

      void computePosr(dxGeom* geom_i)
      {
        // should only be recalced if we need to - ie offset from a body
        dIASSERT(geom_i->offset_posr);  
        dIASSERT(geom_i->body);
        dIASSERT(geom_i->final_posr);

        dMultiply0_331 (geom_i->final_posr->pos,geom_i->body->posr.R,geom_i->offset_posr->pos);
        geom_i->final_posr->pos[0] += geom_i->body->posr.pos[0];
        geom_i->final_posr->pos[1] += geom_i->body->posr.pos[1];
        geom_i->final_posr->pos[2] += geom_i->body->posr.pos[2];
        dMultiply0_333 (geom_i->final_posr->R,geom_i->body->posr.R,geom_i->offset_posr->R);
      }

      // recalculate our new final position if needed
      void recomputePosr(dxGeom* geom_i)
      {
        if (geom_i->gflags & GEOM_POSR_BAD) {
          computePosr(geom_i);
          geom_i->gflags &= ~GEOM_POSR_BAD;
        }
      }

      void recomputeAABB(dxGeom* geom_i) {

        /* std::cout << "HERE A" << std::endl; */
        if (geom_i->gflags & GEOM_AABB_BAD) {
          /* std::cout << "HERE B" << std::endl; */
          // our aabb functions assume final_posr is up to date
          curr_depth_indent ++;
          auto begin_rcPosr = std::chrono::high_resolution_clock::now();
          recomputePosr(geom_i); 
          auto end_rcPosr = std::chrono::high_resolution_clock::now();
          elapsedTime.push_back({currDepthIndent() + "recomputePosr of "+std::to_string(geom_i->type),std::chrono::duration_cast<std::chrono::microseconds>(end_rcPosr - begin_rcPosr).count()});
          curr_depth_indent --;
          /* std::cout << "HERE C" << std::endl; */

          switch (geom_i->type) 
          {
            case (dSimpleSpaceClass):
              {
                curr_depth_indent ++;
                computeAABB((dxSpace*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dHashSpaceClass):
              {
                curr_depth_indent ++;
                computeAABB((dxSpace*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dRayClass):
              {
                curr_depth_indent ++;
                computeAABB((dxRay*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dCylinderClass):
              {
                curr_depth_indent ++;
                computeAABB((dxCylinder*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dSphereClass):
              {
                curr_depth_indent ++;
                computeAABB((dxSphere*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dBoxClass):
              {
                curr_depth_indent ++;
                computeAABB((dxBox*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dCapsuleClass):
              {
                curr_depth_indent ++;
                computeAABB((dxCapsule*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dPlaneClass):
              {
                curr_depth_indent ++;
                computeAABB((dxPlane*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dConvexClass):
              {
                curr_depth_indent ++;
                computeAABB((dxConvex*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dTriMeshClass):
              {
                curr_depth_indent ++;
                computeAABB((dxTriMesh*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dHeightfieldClass):
              {
                curr_depth_indent ++;
                computeAABB((dxHeightfield*)geom_i);
                curr_depth_indent --;
                break;
              }
            case (dGeomTransformClass):
              {
                break;
              }
            default:
              {
                std::cerr << "[UV CAM collisions]: [" << camera_name << "]: Encountered unaddressed geometry type [" << geom_i->type << "]" << std::endl;
                return;
              }
          }
          /* std::cout << "HERE D" << std::endl; */
          geom_i->gflags &= ~GEOM_AABB_BAD;
          /* std::cout << "HERE D2" << std::endl; */
        }

      }

      void dSpaceCollide2 (dxSpace *s1, dxGeom *g2, void *data){
        if (got_obstacle)
          return; //significant acceleration - we don't really care about the range, just if we have line of sight


        auto g1 = (dGeomID)(s1);
        auto begin_rcAABB = std::chrono::high_resolution_clock::now();
        curr_depth_indent ++;
        recomputeAABB(g1);
        curr_depth_indent --;
        auto end_rcAABB = std::chrono::high_resolution_clock::now();
        elapsedTime.push_back({currDepthIndent() + "Top level recomputeAABB",std::chrono::duration_cast<std::chrono::microseconds>(end_rcAABB - begin_rcAABB).count()});
        /* std::cout << "HERE V" << std::endl; */


        // intersect bounding boxes

        /* std::cout << "ray_geomid is " << ray_geomid->type << std::endl; */

        auto begin_collisionTOP = std::chrono::high_resolution_clock::now();
        curr_depth_indent ++;
        for (dxGeom *g=(s1)->first; g; g=g->next) {
          if (got_obstacle)
            return; //significant acceleration - we don't really care about the range, just if we have line of sight

          if (g->type == dRayClass){
            continue;
          }

          /* ode_ray->nextLevel */

          /* dNearCallback* dummy; */
          /* std::cout << "HERE W: g: " << g << std::endl; */

          /* std::cout << "Geometry type [" << g->type << "]" << std::endl; */
          /* if (g->data){ */
          /*   if ( */
          /*       (dGeomGetClass(g) != dGeomTransformClass) && */
          /*       (dGeomGetClass(g) != dFirstSpaceClass) */ 
          /*      ){ */
          /*     auto base_object = static_cast<gazebo::physics::Base*>(dGeomGetData(g)); */
          /*     std::string name = base_object->GetName(); */
          /*     std::cout << "Entity: [" << name << "]" << g << std::endl; */
          /*     if (name.find("inertia_collision")!=std::string::npos){ */
          /*       std::cout << "Let's see..." << g << std::endl; */
          /*     } */
          /*   } */
          /* } */

          if (GEOM_ENABLED(g)){

            auto begin_collisionGEOM = std::chrono::high_resolution_clock::now();

            curr_depth_indent ++;
            collideAABBs (g,g2,data);
            curr_depth_indent --;
            auto end_collisionGEOM = std::chrono::high_resolution_clock::now();
            elapsedTime.push_back({currDepthIndent() + "Object collision calculation between "+ std::to_string(g->type) +" and "+ std::to_string(g2->type)+
                (g->data?" ["+ static_cast<gazebo::physics::Base*>(dGeomGetData(g))->GetName()+"]":"")+
                (g2->data?"-["+ static_cast<gazebo::physics::Base*>(dGeomGetData(g1))->GetName() +"]":""),
                std::chrono::duration_cast<std::chrono::microseconds>(end_collisionGEOM - begin_collisionGEOM).count()
                });
          }
          /* std::cout << "HERE X" << std::endl; */
        }

        curr_depth_indent --;
        auto end_collisionTOP = std::chrono::high_resolution_clock::now();
        elapsedTime.push_back({currDepthIndent() + "Top level collision calculation",std::chrono::duration_cast<std::chrono::microseconds>(end_collisionTOP - begin_collisionTOP).count()});
      }

      void computeAABB(dxSpace* space_i)
      {
        auto begin_cSpace = std::chrono::high_resolution_clock::now();

        if (space_i->first) {
          int i;
          dReal a[6];
          a[0] = dInfinity;
          a[1] = -dInfinity;
          a[2] = dInfinity;
          a[3] = -dInfinity;
          a[4] = dInfinity;
          a[5] = -dInfinity;
          for (dxGeom *g=space_i->first; g; g=g->next) {
            if (got_obstacle)
              return; //significant acceleration - we don't really care about the range, just if we have line of sight

            /* std::cout << "HERE H: g: " << g << std::endl; */

            auto begin_rcAABB = std::chrono::high_resolution_clock::now();
            recomputeAABB(g);
            auto end_rcAABB = std::chrono::high_resolution_clock::now();

            elapsedTime.push_back({currDepthIndent() + "Lower level recomputeAABB of "+ std::to_string(g->type)+
                (g->data?" ["+static_cast<gazebo::physics::Base*>(dGeomGetData(g))->GetName()+"]":""),
                std::chrono::duration_cast<std::chrono::microseconds>(end_rcAABB - begin_rcAABB).count()
                });

            /* std::cout << "HERE H2" << std::endl; */
            for (i=0; i<6; i += 2) if (g->aabb[i] < a[i]) a[i] = g->aabb[i];
            for (i=1; i<6; i += 2) if (g->aabb[i] > a[i]) a[i] = g->aabb[i];
            /* std::cout << "HERE H3" << std::endl; */
          }
          /* std::cout << "HERE I" << std::endl; */
          auto begin_memcopy = std::chrono::high_resolution_clock::now();
          memcpy(space_i->aabb,a,6*sizeof(dReal));
          auto end_memcopy = std::chrono::high_resolution_clock::now();
          elapsedTime.push_back({currDepthIndent() + "Memcopy",std::chrono::duration_cast<std::chrono::microseconds>(end_memcopy - begin_memcopy).count()});
          /* std::cout << "HERE J" << std::endl; */
        }
        else {
          /* std::cout << "HERE K" << std::endl; */
          dSetZero (space_i->aabb,6);
          /* std::cout << "HERE L" << std::endl; */
        }

        auto end_cSpace = std::chrono::high_resolution_clock::now();
        elapsedTime.push_back({currDepthIndent() + "Space type computeAABB",std::chrono::duration_cast<std::chrono::microseconds>(end_cSpace - begin_cSpace).count()});
      }
      void computeAABB(dxRay* ray_i)
      {
        /* std::cout << "HERE M" << std::endl; */
        dVector3 e;
        e[0] = ray_i->final_posr->pos[0] + ray_i->final_posr->R[0*4+2]*(ray_i->length);
        e[1] = ray_i->final_posr->pos[1] + ray_i->final_posr->R[1*4+2]*(ray_i->length);
        e[2] = ray_i->final_posr->pos[2] + ray_i->final_posr->R[2*4+2]*(ray_i->length);

        if (ray_i->final_posr->pos[0] < e[0]){
          ray_i->aabb[0] = ray_i->final_posr->pos[0];
          ray_i->aabb[1] = e[0];
        }
        else{
          ray_i->aabb[0] = e[0];
          ray_i->aabb[1] = ray_i->final_posr->pos[0];
        }

        if (ray_i->final_posr->pos[1] < e[1]){
          ray_i->aabb[2] = ray_i->final_posr->pos[1];
          ray_i->aabb[3] = e[1];
        }
        else{
          ray_i->aabb[2] = e[1];
          ray_i->aabb[3] = ray_i->final_posr->pos[1];
        }

        if (ray_i->final_posr->pos[2] < e[2]){
          ray_i->aabb[4] = ray_i->final_posr->pos[2];
          ray_i->aabb[5] = e[2];
        }
        else{
          ray_i->aabb[4] = e[2];
          ray_i->aabb[5] = ray_i->final_posr->pos[2];
        }
        /* std::cout << "HERE N" << std::endl; */
      }

      void computeAABB(dxCylinder* cylinder_i)
      {
        const dMatrix3& R = cylinder_i->final_posr->R;
        const dVector3& pos = cylinder_i->final_posr->pos;

        dReal xrange = dFabs (R[0] * cylinder_i->radius) +	 dFabs (R[1] * cylinder_i->radius) + REAL(0.5)* dFabs (R[2] * 
            cylinder_i->lz);
        dReal yrange = dFabs (R[4] * cylinder_i->radius) +   dFabs (R[5] * cylinder_i->radius) + REAL(0.5)* dFabs (R[6] * 
            cylinder_i->lz);
        dReal zrange = dFabs (R[8] * cylinder_i->radius) +	 dFabs (R[9] * cylinder_i->radius) + REAL(0.5)* dFabs (R[10] * 
            cylinder_i->lz);
        cylinder_i->aabb[0] = pos[0] - xrange;
        cylinder_i->aabb[1] = pos[0] + xrange;
        cylinder_i->aabb[2] = pos[1] - yrange;
        cylinder_i->aabb[3] = pos[1] + yrange;
        cylinder_i->aabb[4] = pos[2] - zrange;
        cylinder_i->aabb[5] = pos[2] + zrange;
      }

      void computeAABB(dxBox* cylinder_i){
        cylinder_i->computeAABB();
      }

      void computeAABB(dxSphere* sphere_i){
        sphere_i->computeAABB();
      }

      void computeAABB(dxCapsule* capsule_i){
        capsule_i->computeAABB();
      }

      void computeAABB(dxPlane* plane_i){
        plane_i->computeAABB();
      }

      void computeAABB(dxConvex* convex_i){
        convex_i->computeAABB();
      }

      void computeAABB(dxTriMesh* trimesh_i){
        trimesh_i->computeAABB();
      }

      void computeAABB(dxHeightfield* heightfield_i){
        heightfield_i->computeAABB();
      }

      void collideAABBs (dxGeom *g1, dxGeom *g2, void *data)
      {
        if (got_obstacle)
          return; //significant acceleration - we don't really care about the range, just if we have line of sight

        /* std::cout << "HERE O" << std::endl; */
        dIASSERT((g1->gflags & GEOM_AABB_BAD)==0);
        dIASSERT((g2->gflags & GEOM_AABB_BAD)==0);

        /* std::cout << "HERE P" << std::endl; */
        // no contacts if both geoms on the same body, and the body is not 0
        if (g1->body == g2->body && g1->body) return;

        // test if the category and collide bitfields match
        /* std::cout << "HERE Q" << std::endl; */
        if ( ((g1->category_bits & g2->collide_bits) ||
              (g2->category_bits & g1->collide_bits)) == 0) {
          return;
        }

        /* std::cout << "HERE R" << std::endl; */
        // if the bounding boxes are disjoint then don't do anything
        dReal *bounds1 = g1->aabb;
        dReal *bounds2 = g2->aabb;
        if (bounds1[0] > bounds2[1] ||
            bounds1[1] < bounds2[0] ||
            bounds1[2] > bounds2[3] ||
            bounds1[3] < bounds2[2] ||
            bounds1[4] > bounds2[5] ||
            bounds1[5] < bounds2[4]) {
          /* std::cout << "HERE S" << std::endl; */
          return;
        }

        // check if either object is able to prove that it doesn't intersect the
        // AABB of the other
        if (g1->AABBTest (g2,bounds2) == 0) return;
        if (g2->AABBTest (g1,bounds1) == 0) return;

        // the objects might actually intersect - call the space callback function
        nextLevel(data,g1,g2);

      }

      void nextLevel(void *_data, dGeomID _o1, dGeomID _o2)
      {
        if (got_obstacle)
          return; //significant acceleration - we don't really care about the range, just if we have line of sight

        dContactGeom contact;
        ODERayHack::Intersection *inter = nullptr;

        inter = static_cast<Intersection*>(_data);

        // Check space
        if (dGeomIsSpace(_o1))
        {

          curr_depth_indent ++;
          dSpaceCollide2((dxSpace*)(_o1), _o2, _data);
          curr_depth_indent --;
        }
        else
        {
          gazebo::physics::ODECollision *collision1;
          /* , *collision2; */
          gazebo::physics::ODECollision *hitCollision = nullptr;

          // Get pointers to the underlying collisions
          if (dGeomGetClass(_o1) == dGeomTransformClass)
            collision1 =
              static_cast<gazebo::physics::ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o1)));
          else
            collision1 = static_cast<gazebo::physics::ODECollision*>(dGeomGetData(_o1));

          if (collision1->GetName().find("inertia_collision")!=std::string::npos){ // this is the money line
            return;
          }
          if (collision1->GetScopedName().find("rotor")!=std::string::npos){ // this is the money line
            return;
          }
          /* if (dGeomGetClass(_o2) == dGeomTransformClass) */
          /*   collision2 = */
          /*     static_cast<gazebo::physics::ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2))); */
          /* else */
          /*   collision2 = static_cast<gazebo::physics::ODECollision*>(dGeomGetData(_o2)); */

          // Figure out which one is a ray; note that this assumes
          // that the ODE dRayClass is used *soley* by the RayCollision.
          /* if (dGeomGetClass(_o1) == dRayClass) */
          /* { */
          /*   hitCollision = collision2; */
          /*   dGeomRaySetParams(_o1, 0, 0); */
          /*   dGeomRaySetClosestHit(_o1, 1); */
          /* } */
          if (dGeomGetClass(_o2) == dRayClass)
          {
            hitCollision = collision1;
            dGeomRaySetParams(_o2, 0, 0);
            dGeomRaySetClosestHit(_o2, 1);
          }

          if (hitCollision)
          {
            /* std::cout << "00   dSphereClass = 0: " << dSphereClass << std::endl; */
            /* std::cout << "01   dBoxClass: " << dBoxClass << std::endl; */
            /* std::cout << "02   dCapsuleClass: " << dCapsuleClass << std::endl; */
            /* std::cout << "03   dCylinderClass: " << dCylinderClass << std::endl; */
            /* std::cout << "04   dPlaneClass: " << dPlaneClass << std::endl; */
            /* std::cout << "05   dRayClass: " << dRayClass << std::endl; */
            /* std::cout << "06   dConvexClass: " << dConvexClass << std::endl; */
            /* std::cout << "07   dGeomTransformClass: " << dGeomTransformClass << std::endl; */
            /* std::cout << "08   dTriMeshClass: " << dTriMeshClass << std::endl; */
            /* std::cout << "09   dHeightfieldClass: " << dHeightfieldClass << std::endl; */
            /* std::cout << "10   dFirstSpaceClass: " << dFirstSpaceClass << std::endl; */
            /* std::cout << "10   dSimpleSpaceClass: " << dSimpleSpaceClass << std::endl; */
            /* std::cout << "11   dHashSpaceClass: " << dHashSpaceClass << std::endl; */
            /* std::cout << "12   dSweepAndPruneSpaceClass: " << dSweepAndPruneSpaceClass << std::endl; */
            /* std::cout << "13   dQuadTreeSpaceClass: " << dQuadTreeSpaceClass << std::endl; */
            /* std::cout << "13   dLastSpaceClass: " << dLastSpaceClass << std::endl; */
            /* std::cout << "14   dFirstUserClass: " << dFirstUserClass << std::endl; */
            /* std::cout << "17   dLastUserClass: " << dLastUserClass << std::endl; */
            /* std::cout << "18   dGeomNumClasses: " << dGeomNumClasses << std::endl; */

            // Check for ray/collision intersections

            int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));

            if (n > 0)
            {

              /* if (contact.depth < inter->depth) */
              if (contact.depth < led_distance)
              {
                inter->depth = contact.depth;
                inter->name = hitCollision->GetScopedName();
                got_obstacle = true;
              }
            }
          }
        }
      }

    private:
      std::string currDepthIndent(){
        return std::string(2*curr_depth_indent, ' ');
      }

    public:
      int getIntersection(
          ignition::math::Pose3d camera, ignition::math::Pose3d led,
          /* gazebo::physics::RayShapePtr curr_ray, */ 
          Intersection &intersection){

        got_obstacle = false;
        led_distance = (camera.Pos() - led.Pos()).Length();
        ode_ray->SetPoints(camera.Pos(),led.Pos());
        /* auto ode_ray = boost::static_pointer_cast<gazebo::physics::ODERayShape>(curr_ray); */
        auto ray_geomid = (dxSpace*)(ode_ray)->ODEGeomId();


        intersection.depth = INIFINITE_RANGE;
        auto begin       = std::chrono::high_resolution_clock::now();
        curr_depth_indent ++;
        dSpaceCollide2(world_space, ray_geomid, &intersection);
        curr_depth_indent = 0;
        auto end         = std::chrono::high_resolution_clock::now();
        elapsedTime.push_back({"Top level dSpaceCollide2",std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()});
        if (debug){
          for (auto timing : elapsedTime){
            std::cout << "[UV CAM collisions]: ["<< camera_name <<"]: " << timing.first << " took : " << double(timing.second)/1000.0 << " ms" << std::endl;
          }
          elapsedTime.clear();
        }
        return 0;
      }

  };
}

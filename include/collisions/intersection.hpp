#define IS_SPACE(geom) \
  ((geom)->type >= dFirstSpaceClass && (geom)->type <= dLastSpaceClass)

namespace ODERayHack {
  class Intersection
  {
    /// \brief Depth of the ray intersection.
    public: double depth;

            /// \brief Name of the collision object that was hit.
    public: std::string name;
  };

  void computePosr();

  // recalculate our new final position if needed
  void recomputePosr()
  {
    if (gflags & GEOM_POSR_BAD) {
      computePosr();
      gflags &= ~GEOM_POSR_BAD;
    }
  }

  void recomputeAABB(dxGeom* geom_i) {
    std::cout << "HERE A" << std::endl;
    if (geom_i->gflags & GEOM_AABB_BAD) {
      std::cout << "HERE B" << std::endl;
      // our aabb functions assume final_posr is up to date
      geom_i->recomputePosr(); 
      std::cout << "HERE C" << std::endl;
      switch (geom_i->type) {
        case (dSimpleSpaceClass):
          computeAABB((dxSpace*)geom_i);
          break;
        case (dHashSpaceClass):
          computeAABB((dxSpace*)geom_i);
          break;
        case (dRayClass):
          computeAABB((dxRay*)geom_i);
defult:
          return;
      }
      std::cout << "HERE D" << std::endl;
      geom_i->gflags &= ~GEOM_AABB_BAD;
    }
  }

  void computeAABB(dxSpace* space_i)
  {
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
        std::cout << "HERE H: g: " << g << std::endl;
        recomputeAABB(g);
        for (i=0; i<6; i += 2) if (g->aabb[i] < a[i]) a[i] = g->aabb[i];
        for (i=1; i<6; i += 2) if (g->aabb[i] > a[i]) a[i] = g->aabb[i];
      }
      std::cout << "HERE I" << std::endl;
      memcpy(space_i->aabb,a,6*sizeof(dReal));
    }
    else {
      dSetZero (space_i->aabb,6);
    }
  }
  void computeAABB(dxRay* ray_i)
  {
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
  }

inline static void collideAABBs (dxGeom *g1, dxGeom *g2,
			  void *data, dNearCallback *callback)
{
  dIASSERT((g1->gflags & GEOM_AABB_BAD)==0);
  dIASSERT((g2->gflags & GEOM_AABB_BAD)==0);

  // no contacts if both geoms on the same body, and the body is not 0
  if (g1->body == g2->body && g1->body) return;

  // test if the category and collide bitfields match
  if ( ((g1->category_bits & g2->collide_bits) ||
	(g2->category_bits & g1->collide_bits)) == 0) {
    return;
  }

  // if the bounding boxes are disjoint then don't do anything
  dReal *bounds1 = g1->aabb;
  dReal *bounds2 = g2->aabb;
  if (bounds1[0] > bounds2[1] ||
      bounds1[1] < bounds2[0] ||
      bounds1[2] > bounds2[3] ||
      bounds1[3] < bounds2[2] ||
      bounds1[4] > bounds2[5] ||
      bounds1[5] < bounds2[4]) {
    return;
  }

  // check if either object is able to prove that it doesn't intersect the
  // AABB of the other
  if (g1->AABBTest (g2,bounds2) == 0) return;
  if (g2->AABBTest (g1,bounds1) == 0) return;

  // the objects might actually intersect - call the space callback function
  /* callback (data,g1,g2); */
}

}

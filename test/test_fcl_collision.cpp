/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** @author Jia Pan */

#include <gtest/gtest.h>

#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"
#include "fcl/narrowphase/continuous_collision.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"

#include "test_fcl_utility.h"

#include "fcl_resources/config.h"

using namespace fcl;

template<typename BV>
bool collide_Test(const Transform3<typename BV::S>& tf,
                  const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose = true);

template<typename BV>
bool collide_Test2(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose = true);

template<typename BV, typename TraversalNode>
bool collide_Test_Oriented(const Transform3<typename BV::S>& tf,
                           const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                           const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose = true);


template<typename BV>
bool test_collide_func(const Transform3<typename BV::S>& tf,
                       const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method);

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

template<typename S>
std::vector<Contact<S>>& global_pairs()
{
  static std::vector<Contact<S>> static_global_pairs;
  return static_global_pairs;
}

template<typename S>
std::vector<Contact<S>>& global_pairs_now()
{
  static std::vector<Contact<S>> static_global_pairs_now;
  return static_global_pairs_now;
}

template <typename S>
void test_SplineMotion_rotated_spline_collide_test()
{
  fcl::Vector3<S> t[4];
  t[0] = fcl::Vector3<S>(7.5, 8, 0);
  t[1] = fcl::Vector3<S>(4.2, 8, 0);
  t[2] = fcl::Vector3<S>(0.8, 8, 0);
  t[3] = fcl::Vector3<S>(-2.5, 8, 0);

  fcl::Vector3<S> r[4];
  r[0] = fcl::Vector3<S>(0, 0, 3.141593);
  r[1] = fcl::Vector3<S>(0, 0, 3.141593);
  r[2] = fcl::Vector3<S>(0, 0, 3.141593);
  r[3] = fcl::Vector3<S>(0, 0, 3.141593);

  auto motion_a = fcl::make_aligned_shared<fcl::SplineMotion<S>>(
    t[0], t[1], t[2], t[3],
    r[0], r[1], r[2], r[3]);

  t[0] = fcl::Vector3<S>(0.0, 8, 0);
  t[1] = fcl::Vector3<S>(1.25, 8, 0);
  t[2] = fcl::Vector3<S>(3.0, 8, 0);
  t[3] = fcl::Vector3<S>(4.6, 8, 0);

  r[0] = fcl::Vector3<S>(0, 0, 0);
  r[1] = fcl::Vector3<S>(0, 0, 0);
  r[2] = fcl::Vector3<S>(0, 0, 0);
  r[3] = fcl::Vector3<S>(0, 0, 0);

  auto motion_b = fcl::make_aligned_shared<fcl::SplineMotion<S>>(
    t[0], t[1], t[2], t[3],
    r[0], r[1], r[2], r[3]);

  // Test collision with unit spheres
  auto shape_a = std::make_shared<fcl::Sphere<S>>(1.0);
  const auto obj_a = fcl::ContinuousCollisionObject<S>(
    shape_a,
    motion_a);

  auto shape_b = std::make_shared<fcl::Sphere<S>>(1.0);
  const auto obj_b = fcl::ContinuousCollisionObject<S>(
    shape_b,
    motion_b);

  fcl::ContinuousCollisionRequest<S> request;
  request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
  request.gjk_solver_type = fcl::GST_LIBCCD;

  fcl::ContinuousCollisionResult<S> result;
  fcl::collide(&obj_a, &obj_b, request, result);

  EXPECT_TRUE(result.is_collide);
}

GTEST_TEST(FCL_COLLISION, test_SplineMotion_rotated_spline_collide_test)
{
  test_SplineMotion_rotated_spline_collide_test<double>();
}

Eigen::Matrix4d make_M_inv() {
  Eigen::Matrix4d M;
  M.block<1, 4>(0, 0) << 1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0, 0.0;
  M.block<1, 4>(1, 0) << -1.0 / 2.0, 0.0, 1.0 / 2.0, 0.0;
  M.block<1, 4>(2, 0) << 1.0 / 2.0, -1.0, 1.0 / 2.0, 0.0;
  M.block<1, 4>(3, 0) << -1.0 / 6.0, 1.0 / 2.0, -1.0 / 2.0, 1.0 / 6.0;

  return M.inverse();
}

std::array<Eigen::Vector4d, 3> compute_coefficients(const Eigen::Vector3d& x0,
                                                    const Eigen::Vector3d& x1,
                                                    const Eigen::Vector3d& v0,
                                                    const Eigen::Vector3d& v1) {
  std::array<Eigen::Vector4d, 3> coeffs;
  for (int i = 0; i < 3; ++i) {
    // *INDENT-OFF*
    std::size_t si = static_cast<std::size_t>(i);
    coeffs[si][0] = x0[i];                                       // = d
    coeffs[si][1] = v0[i];                                       // = c
    coeffs[si][2] = -v1[i] - 2 * v0[i] + 3 * x1[i] - 3 * x0[i];  // = b
    coeffs[si][3] = v1[i] + v0[i] - 2 * x1[i] + 2 * x0[i];       // = a
    // *INDENT-ON*
  }

  return coeffs;
}

GTEST_TEST(FCL_COLLISION, test_BVH) {
  // Test collision with unit spheres vs
  auto shape_a = std::make_shared<fcl::Sphere<double>>(0.5);

  // auto shape_b = std::make_shared<fcl::Sphere<double>>(0.5);
  // auto shape_b2 = std::make_shared<fcl::Sphere<double>>(0.6);
  fcl::Boxd shape_b(0.5, 0.5, 0.0);
  fcl::Boxd shape_b2(0.6, 0.6, 0.0);

  fcl::Transform3d shape_b2_offset;
  shape_b2_offset.setIdentity();
  shape_b2_offset.pretranslate(Eigen::Vector3d(0, -1.0, 0));

  auto shape_b_bvh = fcl::make_aligned_shared<fcl::BVHModel<fcl::OBBRSSd>>();
#if 0
  auto box_to_triangle_vertices =
      [](const fcl::Boxd& box, const fcl::Transform3d& pose,
         std::vector<fcl::Vector3d>& vertices_out,
         std::vector<fcl::Triangle>& triangles_out)
  {
    double x_length = box.side[0];
    double y_length = box.side[1];

    vertices_out.resize(4);
    vertices_out[0] = fcl::Vector3d(-0.5 * x_length, -0.5 * y_length, 0.0);
    vertices_out[1] = fcl::Vector3d(0.5 * x_length, -0.5 * y_length, 0.0);
    vertices_out[2] = fcl::Vector3d(-0.5 * x_length, 0.5 * y_length, 0.0);
    vertices_out[3] = fcl::Vector3d(0.5 * x_length, 0.5 * y_length, 0.0);

    triangles_out.resize(2);
    triangles_out[0].set(0, 1, 2);
    triangles_out[1].set(1, 3, 2);

    for (unsigned int i = 0; i < vertices_out.size(); ++i)
      vertices_out[i] = pose * vertices_out[i];
    };

  shape_b_bvh->beginModel();
  // add shape
  {
    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangle_indices;

    fcl::Transform3d identity;
    identity.setIdentity();
    box_to_triangle_vertices(shape_b, identity, vertices, triangle_indices);
    int r = shape_b_bvh->addSubModel(vertices, triangle_indices);
    if (r != fcl::BVH_OK) printf("failed#1\n");

    // vertices.clear();
    // triangle_indices.clear();
    // box_to_triangle_vertices(shape_b2, shape_b2_offset, vertices,
    // triangle_indices); r = shape_b_bvh->addSubModel(vertices,
    // triangle_indices); if (r != fcl::BVH_OK)
    //   printf("failed#2");
  }
  shape_b_bvh->endModel();
#else
  fcl::Transform3d ident;
  ident.setIdentity();
  int res = fcl::generateBVHModel(*shape_b_bvh, shape_b, ident, fcl::FinalizeModel::DO);
  //res = fcl::generateBVHModel(*shape_b_bvh, shape_b2, shape_b2_offset, fcl::FinalizeModel::DO);
#endif

#if 1
  // spline motion with same straight line motion. ends up with with an stackoverflow SEH with distanceRecurse going infinate
  std::shared_ptr<fcl::SplineMotion<double>> motion_a, motion_b;
  fcl::Vector3d t[4];
  t[0] = fcl::Vector3d(7.5, 8, 0);
  t[1] = fcl::Vector3d(4.2, 8, 0);
  t[2] = fcl::Vector3d(0.8, 8, 0);
  t[3] = fcl::Vector3d(-2.5, 8, 0);

  fcl::Vector3d r[4];
  r[0] = fcl::Vector3d(0, 0, 0);
  r[1] = fcl::Vector3d(0, 0, 0);
  r[2] = fcl::Vector3d(0, 0, 0);
  r[3] = fcl::Vector3d(0, 0, 0);

  motion_a = fcl::make_aligned_shared<fcl::SplineMotion<double>>(
      t[0], t[1], t[2], t[3], r[0], r[1], r[2], r[3]);

  t[0] = fcl::Vector3d(0.0, 8, 0);
  t[1] = fcl::Vector3d(1.25, 8, 0);
  t[2] = fcl::Vector3d(3.0, 8, 0);
  t[3] = fcl::Vector3d(4.6, 8, 0);

  r[0] = fcl::Vector3d(0, 0, 0);
  r[1] = fcl::Vector3d(0, 0, 0);
  r[2] = fcl::Vector3d(0, 0, 0);
  r[3] = fcl::Vector3d(0, 0, 0);

  motion_b = fcl::make_aligned_shared<fcl::SplineMotion<double>>(
      t[0], t[1], t[2], t[3], r[0], r[1], r[2], r[3]);
#else
  // interp motion with the same straight line motion. works as expected.
  std::shared_ptr<fcl::InterpMotion<double>> motion_a, motion_b;

  fcl::Transform3d a1;
  a1.setIdentity();
  a1.translation() = fcl::Vector3<double>(7.5, 8, 0);

  fcl::Transform3d a2;
  a2.setIdentity();
  a2.translation() = fcl::Vector3<double>(- 2.5, 8, 0);
  motion_a = fcl::make_aligned_shared<fcl::InterpMotion<double>>(a1, a2);

  fcl::Transform3d b1;
  b1.setIdentity();
  b1.translation() = fcl::Vector3<double>(0, 8, 0);

  fcl::Transform3d b2;
  b2.setIdentity();
  b2.translation() = fcl::Vector3<double>(4.6, 8, 0 );
  motion_b = fcl::make_aligned_shared<fcl::InterpMotion<double>>(b1, b2);
#endif

  fcl::ContinuousCollisionRequest<double> request;
  request.ccd_solver_type = fcl::CCDC_CONSERVATIVE_ADVANCEMENT;
  request.gjk_solver_type = fcl::GST_LIBCCD;

  // reset the motions
  motion_a->integrate(0.0);
  motion_b->integrate(0.0);

  const auto obj_a = fcl::ContinuousCollisionObject<double>(shape_a, motion_a);
  const auto obj_b = fcl::ContinuousCollisionObject<double>(shape_b_bvh, motion_b);

  printf("asdasd\n");
  // test for collision
  fcl::Transform3d identity_offset;
  identity_offset.setIdentity();
  fcl::ContinuousCollisionResultd result;
  fcl::collide(&obj_a, &obj_b, request, result);

  if (result.is_collide)
    printf("collide\n");
  else
    printf("nocollide\n");
}

template <typename S>
void test_OBB_Box_test()
{
  S r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  aligned_vector<Transform3<S>> rotate_transform;
  test::generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB<S> aabb1;
  aabb1.min_ = Vector3<S>(-600, -600, -600);
  aabb1.max_ = Vector3<S>(600, 600, 600);

  OBB<S> obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box<S> box1;
  Transform3<S> box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  S extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  aligned_vector<Transform3<S>> transforms;
  test::generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    AABB<S> aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    OBB<S> obb2;
    convertBV(aabb, transforms[i], obb2);

    Box<S> box2;
    Transform3<S> box2_tf;
    constructBox(aabb, transforms[i], box2, box2_tf);

    detail::GJKSolver_libccd<S> solver;

    bool overlap_obb = obb1.overlap(obb2);
    bool overlap_box = solver.shapeIntersect(box1, box1_tf, box2, box2_tf, nullptr);

    EXPECT_TRUE(overlap_obb == overlap_box);
  }
}

template <typename S>
void test_OBB_shape_test()
{
  S r_extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  aligned_vector<Transform3<S>> rotate_transform;
  test::generateRandomTransforms(r_extents, rotate_transform, 1);

  AABB<S> aabb1;
  aabb1.min_ = Vector3<S>(-600, -600, -600);
  aabb1.max_ = Vector3<S>(600, 600, 600);

  OBB<S> obb1;
  convertBV(aabb1, rotate_transform[0], obb1);
  Box<S> box1;
  Transform3<S> box1_tf;
  constructBox(aabb1, rotate_transform[0], box1, box1_tf);

  S extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  aligned_vector<Transform3<S>> transforms;
  test::generateRandomTransforms(extents, transforms, n);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    S len = (aabb1.max_[0] - aabb1.min_[0]) * 0.5;
    OBB<S> obb2;
    detail::GJKSolver_libccd<S> solver;

    {
      Sphere<S> sphere(len);
      computeBV(sphere, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_sphere = solver.shapeIntersect(box1, box1_tf, sphere, transforms[i], nullptr);
      EXPECT_TRUE(overlap_obb >= overlap_sphere);
    }

    {
      Ellipsoid<S> ellipsoid(len, len, len);
      computeBV(ellipsoid, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_ellipsoid = solver.shapeIntersect(box1, box1_tf, ellipsoid, transforms[i], nullptr);
      EXPECT_TRUE(overlap_obb >= overlap_ellipsoid);
    }

    {
      Capsule<S> capsule(len, 2 * len);
      computeBV(capsule, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_capsule = solver.shapeIntersect(box1, box1_tf, capsule, transforms[i], nullptr);
      EXPECT_TRUE(overlap_obb >= overlap_capsule);
    }

    {
      Cone<S> cone(len, 2 * len);
      computeBV(cone, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_cone = solver.shapeIntersect(box1, box1_tf, cone, transforms[i], nullptr);
      EXPECT_TRUE(overlap_obb >= overlap_cone);
    }

    {
      Cylinder<S> cylinder(len, 2 * len);
      computeBV(cylinder, transforms[i], obb2);

      bool overlap_obb = obb1.overlap(obb2);
      bool overlap_cylinder = solver.shapeIntersect(box1, box1_tf, cylinder, transforms[i], nullptr);
      EXPECT_TRUE(overlap_obb >= overlap_cylinder);
    }
  }
}

template <typename S>
void test_OBB_AABB_test()
{
  S extents[] = {-1000, -1000, -1000, 1000, 1000, 1000};
  std::size_t n = 1000;

  aligned_vector<Transform3<S>> transforms;
  test::generateRandomTransforms(extents, transforms, n);

  AABB<S> aabb1;
  aabb1.min_ = Vector3<S>(-600, -600, -600);
  aabb1.max_ = Vector3<S>(600, 600, 600);

  OBB<S> obb1;
  convertBV(aabb1, Transform3<S>::Identity(), obb1);

  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    AABB<S> aabb;
    aabb.min_ = aabb1.min_ * 0.5;
    aabb.max_ = aabb1.max_ * 0.5;

    AABB<S> aabb2 = translate(aabb, transforms[i].translation());

    OBB<S> obb2;
    convertBV(aabb, Transform3<S>(Translation3<S>(transforms[i].translation())), obb2);

    bool overlap_aabb = aabb1.overlap(aabb2);
    bool overlap_obb = obb1.overlap(obb2);
    if(overlap_aabb != overlap_obb)
    {
      std::cout << aabb1.min_.transpose() << " " << aabb1.max_.transpose() << std::endl;
      std::cout << aabb2.min_.transpose() << " " << aabb2.max_.transpose() << std::endl;
      std::cout << obb1.To.transpose() << " " << obb1.extent.transpose() << " " << obb1.axis.col(0).transpose() << " " << obb1.axis.col(1).transpose() << " " << obb1.axis.col(2).transpose() << std::endl;
      std::cout << obb2.To.transpose() << " " << obb2.extent.transpose() << " " << obb2.axis.col(0).transpose() << " " << obb2.axis.col(1).transpose() << " " << obb2.axis.col(2).transpose() << std::endl;
    }

    EXPECT_TRUE(overlap_aabb == overlap_obb);
  }
  std::cout << std::endl;
}

template <typename S>
void test_mesh_mesh()
{
  std::vector<Vector3<S>> p1, p2;
  std::vector<Triangle> t1, t2;

  test::loadOBJFile(TEST_RESOURCES_DIR"/env.obj", p1, t1);
  test::loadOBJFile(TEST_RESOURCES_DIR"/rob.obj", p2, t2);

  aligned_vector<Transform3<S>> transforms;
  S extents[] = {-3000, -3000, 0, 3000, 3000, 3000};
#ifdef NDEBUG
  std::size_t n = 10;
#else
  std::size_t n = 1;
#endif
  bool verbose = false;

  test::generateRandomTransforms(extents, transforms, n);

  // collision
  for(std::size_t i = 0; i < transforms.size(); ++i)
  {
    global_pairs<S>().clear();
    global_pairs_now<S>().clear();

    collide_Test<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);

    collide_Test<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 24> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 18> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<KDOP<S, 16> >(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBB<S>, detail::MeshCollisionTraversalNodeOBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);

    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBB<S>, detail::MeshCollisionTraversalNodeOBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);

    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBB<S>, detail::MeshCollisionTraversalNodeOBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<RSS<S>, detail::MeshCollisionTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<RSS<S>, detail::MeshCollisionTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<RSS<S>, detail::MeshCollisionTraversalNodeRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<RSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<OBB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<AABB<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }


    collide_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<kIOS<S>, detail::MeshCollisionTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<kIOS<S>, detail::MeshCollisionTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<kIOS<S>, detail::MeshCollisionTraversalNodekIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<kIOS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test2<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<S>, detail::MeshCollisionTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<S>, detail::MeshCollisionTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    collide_Test_Oriented<OBBRSS<S>, detail::MeshCollisionTraversalNodeOBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER, verbose);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_MEDIAN);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }

    test_collide_func<OBBRSS<S>>(transforms[i], p1, t1, p2, t2, detail::SPLIT_METHOD_BV_CENTER);
    EXPECT_TRUE(global_pairs<S>().size() == global_pairs_now<S>().size());
    for(std::size_t j = 0; j < global_pairs<S>().size(); ++j)
    {
      EXPECT_TRUE(global_pairs<S>()[j].b1 == global_pairs_now<S>()[j].b1);
      EXPECT_TRUE(global_pairs<S>()[j].b2 == global_pairs_now<S>()[j].b2);
    }
  }
}

GTEST_TEST(FCL_COLLISION, OBB_Box_test)
{
//  test_OBB_Box_test<float>();
  // Disabled for particular configurations: macOS + release + double (see #202)
#if !defined(FCL_OS_MACOS) || !defined(NDEBUG)
  test_OBB_Box_test<double>();
#endif
}

GTEST_TEST(FCL_COLLISION, OBB_shape_test)
{
//  test_OBB_shape_test<float>();
  test_OBB_shape_test<double>();
}

GTEST_TEST(FCL_COLLISION, OBB_AABB_test)
{
//  test_OBB_AABB_test<float>();
  test_OBB_AABB_test<double>();
}

GTEST_TEST(FCL_COLLISION, mesh_mesh)
{
//  test_mesh_mesh<float>();
  test_mesh_mesh<double>();
}

template<typename BV>
bool collide_Test2(const Transform3<typename BV::S>& tf,
                   const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                   const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));

  std::vector<Vector3<S>> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i)
  {
    vertices1_new[i] = tf * vertices1[i];
  }


  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<S> pose1 = Transform3<S>::Identity();
  Transform3<S> pose2 = Transform3<S>::Identity();

  CollisionResult<S> local_result;
  detail::MeshCollisionTraversalNode<BV> node;

  if(!detail::initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest<S>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);


  if(local_result.numContacts() > 0)
  {
    if(global_pairs<S>().size() == 0)
    {
      local_result.getContacts(global_pairs<S>());
      std::sort(global_pairs<S>().begin(), global_pairs<S>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<S>());
      std::sort(global_pairs_now<S>().begin(), global_pairs_now<S>().end());
    }


    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

template<typename BV>
bool collide_Test(const Transform3<typename BV::S>& tf,
                  const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                  const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<S> pose1(tf);
  Transform3<S> pose2 = Transform3<S>::Identity();

  CollisionResult<S> local_result;
  detail::MeshCollisionTraversalNode<BV> node;

  if(!detail::initialize<BV>(node, m1, pose1, m2, pose2,
                     CollisionRequest<S>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);


  if(local_result.numContacts() > 0)
  {
    if(global_pairs<S>().size() == 0)
    {
      local_result.getContacts(global_pairs<S>());
      std::sort(global_pairs<S>().begin(), global_pairs<S>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<S>());
      std::sort(global_pairs_now<S>().begin(), global_pairs_now<S>().end());
    }

    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}

template<typename BV, typename TraversalNode>
bool collide_Test_Oriented(const Transform3<typename BV::S>& tf,
                           const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                           const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method, bool verbose)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<S> pose1(tf);
  Transform3<S> pose2 = Transform3<S>::Identity();

  CollisionResult<S> local_result;
  TraversalNode node;
  if(!initialize(node, (const BVHModel<BV>&)m1, pose1, (const BVHModel<BV>&)m2, pose2,
                 CollisionRequest<S>(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  collide(&node);

  if(local_result.numContacts() > 0)
  {
    if(global_pairs<S>().size() == 0)
    {
      local_result.getContacts(global_pairs<S>());
      std::sort(global_pairs<S>().begin(), global_pairs<S>().end());
    }
    else
    {
      local_result.getContacts(global_pairs_now<S>());
      std::sort(global_pairs_now<S>().begin(), global_pairs_now<S>().end());
    }

    if(verbose)
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return true;
  }
  else
  {
    if(verbose) std::cout << "collision free " << std::endl;
    if(verbose) std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    return false;
  }
}


template<typename BV>
bool test_collide_func(const Transform3<typename BV::S>& tf,
                       const std::vector<Vector3<typename BV::S>>& vertices1, const std::vector<Triangle>& triangles1,
                       const std::vector<Vector3<typename BV::S>>& vertices2, const std::vector<Triangle>& triangles2, detail::SplitMethodType split_method)
{
  using S = typename BV::S;

  BVHModel<BV> m1;
  BVHModel<BV> m2;
  m1.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new detail::BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  Transform3<S> pose1(tf);
  Transform3<S> pose2 = Transform3<S>::Identity();

  std::vector<Contact<S>> contacts;

  CollisionRequest<S> request(num_max_contacts, enable_contact);
  CollisionResult<S> result;
  int num_contacts = collide(&m1, pose1, &m2, pose2, request, result);

  result.getContacts(contacts);

  global_pairs_now<S>().resize(num_contacts);

  for(int i = 0; i < num_contacts; ++i)
  {
    global_pairs_now<S>()[i].b1 = contacts[i].b1;
    global_pairs_now<S>()[i].b2 = contacts[i].b2;
  }

  std::sort(global_pairs_now<S>().begin(), global_pairs_now<S>().end());

  if(num_contacts > 0) return true;
  else return false;
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

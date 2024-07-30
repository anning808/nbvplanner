/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_gazebo_plugins/gazebo_octomap_plugin.h"

#include <gazebo/common/Time.hh>
// #include <gazebo/math/Vector3.hh>
#include <ignition/math.hh>
#include <ignition/math/Vector3.hh>
#include <octomap_msgs/conversions.h>

namespace gazebo {

OctomapFromGazeboWorld::~OctomapFromGazeboWorld() {
  delete octomap_;
  octomap_ = NULL;
}

void OctomapFromGazeboWorld::Load(physics::WorldPtr _parent,
                                  sdf::ElementPtr _sdf) {
  world_ = _parent;
  std::string service_name = "world/get_octomap";
  gzlog << "Advertising service: " << service_name << std::endl;
  srv_ = node_handle_.advertiseService(
      service_name, &OctomapFromGazeboWorld::ServiceCallback, this);
}

bool OctomapFromGazeboWorld::ServiceCallback(
    rotors_comm::Octomap::Request& req, rotors_comm::Octomap::Response& res) {
  std::cout << "Creating octomap with origin at (" << req.bounding_box_origin.x
            << ", " << req.bounding_box_origin.y << ", "
            << req.bounding_box_origin.z << "), and bounding box lengths ("
            << req.bounding_box_lengths.x << ", " << req.bounding_box_lengths.y
            << ", " << req.bounding_box_lengths.z
            << "), and leaf size: " << req.leaf_size << ".\n";
  CreateOctomap(req);
  if (req.filename != "") {
    if (octomap_) {
      std::string path = req.filename;
      octomap_->writeBinary(path);
      std::cout << std::endl
                << "Octree saved as " << path << std::endl;
    } else {
      std::cout << "The octree is NULL. Will not save that." << std::endl;
    }
  }
  common::Time now = world_->SimTime();
  res.map.header.frame_id = "world";
  res.map.header.stamp = ros::Time(now.sec, now.nsec);

  if (octomap_msgs::binaryMapToMsgData(*octomap_, res.map.data)) {
    res.map.id = "OcTree";
    res.map.binary = true;
    res.map.resolution = octomap_->getResolution();
  } else {
    ROS_ERROR("Error serializing OctoMap");
  }
  std::cout << "Publishing Octomap." << std::endl;
  return true;
}

void OctomapFromGazeboWorld::FloodFill(
    const ignition::math::Vector3<double>& seed_point, const ignition::math::Vector3<double>& bounding_box_origin,
    const ignition::math::Vector3<double>& bounding_box_lengths, const double leaf_size) {
  octomap::OcTreeNode* seed =
      octomap_->search(seed_point.X(), seed_point.Y(), seed_point.Z());
  // do nothing if point occupied
  if (seed != NULL && seed->getOccupancy()) return;

  std::stack<ignition::math::Vector3<double>> to_check;
  to_check.push(ignition::math::Vector3<double>(seed_point.X(), seed_point.Y(), seed_point.Z()));

  while (to_check.size() > 0) {
    ignition::math::Vector3<double> p = to_check.top();
    octomap::point3d point(p.X(), p.Y(), p.Z());
    if ((p.X() > bounding_box_origin.X() - bounding_box_lengths.X() / 2) &&
        (p.X() < bounding_box_origin.X() + bounding_box_lengths.X() / 2) &&
        (p.Y() > bounding_box_origin.Y() - bounding_box_lengths.Y() / 2) &&
        (p.Y() < bounding_box_origin.Y() + bounding_box_lengths.Y() / 2) &&
        (p.Z() > bounding_box_origin.Z() - bounding_box_lengths.Z() / 2) &&
        (p.Z() < bounding_box_origin.Z() + bounding_box_lengths.Z() / 2) &&
        (!octomap_->search(point))) {
      octomap_->setNodeValue(point, 0);
      to_check.pop();
      to_check.push(ignition::math::Vector3<double>(p.X() + leaf_size, p.Y(), p.Z()));
      to_check.push(ignition::math::Vector3<double>(p.X() - leaf_size, p.Y(), p.Z()));
      to_check.push(ignition::math::Vector3<double>(p.X(), p.Y() + leaf_size, p.Z()));
      to_check.push(ignition::math::Vector3<double>(p.X(), p.Y() - leaf_size, p.Z()));
      to_check.push(ignition::math::Vector3<double>(p.X(), p.Y(), p.Z() + leaf_size));
      to_check.push(ignition::math::Vector3<double>(p.X(), p.Y(), p.Z() - leaf_size));

    } else {
      to_check.pop();
    }
  }
}

bool OctomapFromGazeboWorld::CheckIfInterest(const ignition::math::Vector3<double>& central_point,
                                             gazebo::physics::RayShapePtr ray,
                                             const double leaf_size) {
  ignition::math::Vector3<double> start_point = central_point;
  ignition::math::Vector3<double> end_point = central_point;

  double dist;
  std::string entity_name;

  start_point.X() += leaf_size / 2;
  end_point.X() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  if (dist <= leaf_size) return true;

  start_point = central_point;
  end_point = central_point;
  start_point.Y() += leaf_size / 2;
  end_point.Y() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  if (dist <= leaf_size) return true;

  start_point = central_point;
  end_point = central_point;
  start_point.Z() += leaf_size / 2;
  end_point.Z() -= leaf_size / 2;
  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity_name);

  if (dist <= leaf_size) return true;

  return false;
}

void OctomapFromGazeboWorld::CreateOctomap(
    const rotors_comm::Octomap::Request& msg) {
  const double epsilon = 0.00001;
  const int far_away = 100000;
  ignition::math::Vector3<double> bounding_box_origin(msg.bounding_box_origin.x,
                                    msg.bounding_box_origin.y,
                                    msg.bounding_box_origin.z);
  // epsilion prevents undefiened behaviour if a point is inserted exactly
  // between two octomap cells
  ignition::math::Vector3<double> bounding_box_lengths(msg.bounding_box_lengths.x + epsilon,
                                     msg.bounding_box_lengths.y + epsilon,
                                     msg.bounding_box_lengths.z + epsilon);
  double leaf_size = msg.leaf_size;
  octomap_ = new octomap::OcTree(leaf_size);
  octomap_->clear();
  octomap_->setProbHit(0.7);
  octomap_->setProbMiss(0.4);
  octomap_->setClampingThresMin(0.12);
  octomap_->setClampingThresMax(0.97);
  octomap_->setOccupancyThres(0.7);

  gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
          engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Rasterizing world and checking collisions" << std::endl;

  for (double x =
           leaf_size / 2 + bounding_box_origin.X() - bounding_box_lengths.X() / 2;
       x < bounding_box_origin.X() + bounding_box_lengths.X() / 2; x += leaf_size) {
    int progress =
        round(100 * (x + bounding_box_lengths.X() / 2 - bounding_box_origin.X()) /
              bounding_box_lengths.X());
    std::cout << "\rPlacing model edges into octomap... " << progress
              << "%                 ";

    for (double y =
             leaf_size / 2 + bounding_box_origin.Y() - bounding_box_lengths.Y() / 2;
         y < bounding_box_origin.Y() + bounding_box_lengths.Y() / 2;
         y += leaf_size) {
      for (double z = leaf_size / 2 + bounding_box_origin.Z() -
                      bounding_box_lengths.Z() / 2;
           z < bounding_box_origin.Z() + bounding_box_lengths.Z() / 2;
           z += leaf_size) {
        ignition::math::Vector3<double> point(x, y, z);
        if (CheckIfInterest(point, ray, leaf_size)) {
          octomap_->setNodeValue(x, y, z, 1);
        }
      }
    }
  }
  octomap_->prune();
  octomap_->updateInnerOccupancy();

  // flood fill from top and bottom
  std::cout << "\rFlood filling freespace...                                  ";
  FloodFill(ignition::math::Vector3<double>(bounding_box_origin.X() + leaf_size / 2,
                          bounding_box_origin.Y() + leaf_size / 2,
                          bounding_box_origin.Z() + bounding_box_lengths.Z() / 2 -
                              leaf_size / 2),
            bounding_box_origin, bounding_box_lengths, leaf_size);
  FloodFill(ignition::math::Vector3<double>(bounding_box_origin.X() + leaf_size / 2,
                          bounding_box_origin.Y() + leaf_size / 2,
                          bounding_box_origin.Z() - bounding_box_lengths.Z() / 2 +
                              leaf_size / 2),
            bounding_box_origin, bounding_box_lengths, leaf_size);

  octomap_->prune();
  octomap_->updateInnerOccupancy();

  // set unknown to filled
  for (double x =
           leaf_size / 2 + bounding_box_origin.X() - bounding_box_lengths.X() / 2;
       x < bounding_box_origin.X() + bounding_box_lengths.X() / 2; x += leaf_size) {
    int progress =
        round(100 * (x + bounding_box_lengths.X() / 2 - bounding_box_origin.X()) /
              bounding_box_lengths.X());
    std::cout << "\rFilling closed spaces... " << progress << "%              ";

    for (double y =
             leaf_size / 2 + bounding_box_origin.Y() - bounding_box_lengths.Y() / 2;
         y < bounding_box_origin.Y() + bounding_box_lengths.Y() / 2;
         y += leaf_size) {
      for (double z = leaf_size / 2 + bounding_box_origin.Z() -
                      bounding_box_lengths.Z() / 2;
           z < bounding_box_origin.Z() + bounding_box_lengths.Z() / 2;
           z += leaf_size) {
        octomap::OcTreeNode* seed = octomap_->search(x, y, z);
        if (!seed) octomap_->setNodeValue(x, y, z, 1);
      }
    }
  }

  octomap_->prune();
  octomap_->updateInnerOccupancy();

  std::cout << "\rOctomap generation completed                  " << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OctomapFromGazeboWorld)
}

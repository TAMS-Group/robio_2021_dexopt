// (c) 2020-2021 Philipp Ruppel

#include <tractor/tractor.h>

#include "dexlearn.h"

#include "dexenv_grasp.h"
#include "dexenv_push.h"
#include "dexenv_turn.h"

#include "common.h"
#include "goals.h"
#include "neural.h"
#include "physics5.h"

static constexpr size_t inner_batch_size = 8;
static constexpr size_t outer_batch_size = 2;

typedef double ValueSingle;
typedef tractor::Batch<ValueSingle, inner_batch_size> ValueBatch;

typedef tractor::Var<ValueSingle> ScalarSingle;
typedef tractor::GeometryFast<ScalarSingle> GeometrySingle;

typedef tractor::Var<ValueBatch> ScalarBatch;
typedef tractor::GeometryFast<ScalarBatch> GeometryBatch;

int main(int argc, char **argv) {

  std::vector<std::shared_ptr<tractor::DexEnv<ValueSingle, ValueBatch>>> envs =
      {
          std::make_shared<tractor::DexEnvGrasp<ValueSingle, ValueBatch>>(),
          std::make_shared<tractor::DexEnvTurn<ValueSingle, ValueBatch>>(),
          std::make_shared<tractor::DexEnvPush<ValueSingle, ValueBatch>>(),
      };

  if (argc < 3) {
    std::cerr << "USAGE: dexopt <env> <command> [<solver>]" << std::endl;
    return -1;
  }
  std::string envname = argv[1];
  std::string command = argv[2];

  std::string solvername = "sq";
  if (argc > 3) {
    solvername = argv[3];
  }

  ros::WallDuration training_time(60 * 5);

  std::string filename = "weights-" + envname + "-" + solvername + ".dat";

  std::shared_ptr<tractor::DexEnv<ValueSingle, ValueBatch>> env;
  for (auto &e : envs) {
    if (e->info().name == envname) {
      env = e;
    }
  }
  if (env == nullptr) {
    throw std::runtime_error("unknown env " + envname);
  }

  std::string robot = "robot_description";

  ros::init(argc, argv, "tractor_test_sim", 0);
  ros::NodeHandle node_handle;

  std::string group_robot = "robot";
  std::string group_all = "all";

  RobotTrajectoryPublisher robot_trajectory_publisher;

  std::string robot_name;
  node_handle.param(robot, robot_name, std::string("???"));

  ros::AsyncSpinner spinner(4);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader(robot, false);
  auto robot_model = robot_model_loader.getModel();

  auto engine = std::make_shared<tractor::SimpleEngine>();
  // auto engine = std::make_shared<tractor::LoopEngine>();
  // auto engine = std::make_shared<tractor::JITEngine>();

  ros::Publisher visualization_publisher =
      node_handle.advertise<visualization_msgs::MarkerArray>(
          "/tractor/visualization", 10, true);

  planning_scene::PlanningScene planning_scene(robot_model);

  auto acm2 = planning_scene.getAllowedCollisionMatrix();
  {
    for (auto &a : robot_model->getLinkModelNames()) {
      for (auto &b : robot_model->getLinkModelNames()) {
        acm2.setEntry(a, b, true);
      }
    }

    acm2.setEntry("lfdistal", "rfmiddle", false);
    acm2.setEntry("rfdistal", "lfmiddle", false);

    acm2.setEntry("ffdistal", "mfdistal", false);
    acm2.setEntry("mfdistal", "rfdistal", false);
    acm2.setEntry("rfdistal", "lfdistal", false);
    acm2.setEntry("lfdistal", "thdistal", false);

    acm2.setEntry("ffmiddle", "mfmiddle", false);
    acm2.setEntry("mfmiddle", "rfmiddle", false);
    acm2.setEntry("rfmiddle", "lfmiddle", false);
    acm2.setEntry("lfmiddle", "thmiddle", false);

    acm2.setEntry("ffproximal", "mfproximal", false);
    acm2.setEntry("mfproximal", "rfproximal", false);
    acm2.setEntry("rfproximal", "lfproximal", false);
    acm2.setEntry("lfproximal", "thproximal", false);

    acm2.setEntry("object", "ffdistal", false);
    acm2.setEntry("object", "mfdistal", false);
    acm2.setEntry("object", "rfdistal", false);
    acm2.setEntry("object", "lfdistal", false);
    acm2.setEntry("object", "thdistal", false);

    acm2.setEntry("object", "ffmiddle", false);
    acm2.setEntry("object", "mfmiddle", false);
    acm2.setEntry("object", "rfmiddle", false);
    acm2.setEntry("object", "lfmiddle", false);
    acm2.setEntry("object", "thmiddle", false);

    acm2.setEntry("object", "ffproximal", false);
    acm2.setEntry("object", "mfproximal", false);
    acm2.setEntry("object", "rfproximal", false);
    acm2.setEntry("object", "lfproximal", false);
    acm2.setEntry("object", "thproximal", false);

    acm2.setEntry("floor", "ffdistal", false);
    acm2.setEntry("floor", "mfdistal", false);
    acm2.setEntry("floor", "rfdistal", false);
    acm2.setEntry("floor", "lfdistal", false);
    acm2.setEntry("floor", "thdistal", false);

    acm2.setEntry("floor", "forearm", false);
    acm2.setEntry("floor", "arm_wrist_3_link", false);
    acm2.setEntry("floor", "arm_wrist_2_link", false);
    acm2.setEntry("floor", "arm_wrist_1_link", false);

    acm2.setEntry("object", "palm", false);

    acm2.setEntry("object", "floor", false);
  }

  auto joint_names =
      robot_model->getJointModelGroup(group_robot)->getVariableNames();

  std::shared_ptr<tractor::Solver> solver;

  if (solvername == "sq") {
    auto s = std::make_shared<tractor::LeastSquaresSolver<ValueSingle>>(engine);
    s->_regularization = 0.1;
    s->_max_linear_iterations = 100;
    s->_step_scaling = 0.5;
    s->setTimeout(1, false);
    s->setTolerance(1e-9);
    solver = s;
  } else

      if (solvername == "sq03") {
    auto s = std::make_shared<tractor::LeastSquaresSolver<ValueSingle>>(engine);
    s->_regularization = 0.3;
    s->_max_linear_iterations = 100;
    s->_step_scaling = 0.5;
    s->setTimeout(1, false);
    s->setTolerance(1e-9);
    solver = s;
  } else

      if (solvername == "sq1") {
    auto s = std::make_shared<tractor::LeastSquaresSolver<ValueSingle>>(engine);
    s->_regularization = 1;
    s->_max_linear_iterations = 100;
    s->_step_scaling = 0.5;
    s->setTimeout(1, false);
    s->setTolerance(1e-9);
    solver = s;
  } else

      if (solvername == "gd01") {
    solver = std::make_shared<tractor::GradientDescentSolver<ValueSingle>>(
        engine, 0.1, 0.0);
    solver->setTimeout(1, true);
  } else

      if (solvername == "gd001") {
    solver = std::make_shared<tractor::GradientDescentSolver<ValueSingle>>(
        engine, 0.01, 0.0);
    solver->setTimeout(1, true);
  } else

      if (solvername == "gd0001") {
    solver = std::make_shared<tractor::GradientDescentSolver<ValueSingle>>(
        engine, 0.001, 0.0);
    solver->setTimeout(1, true);
  } else

      if (solvername == "gd00001") {
    solver = std::make_shared<tractor::GradientDescentSolver<ValueSingle>>(
        engine, 0.0001, 0.0);
    solver->setTimeout(1, true);
  } else

      if (solvername == "gd000001") {
    solver = std::make_shared<tractor::GradientDescentSolver<ValueSingle>>(
        engine, 0.00001, 0.0);
    solver->setTimeout(1, true);
  } else

  {
    throw std::runtime_error("unknown solver " + solvername);
  }

  tractor::DexLearn<ValueSingle, ValueBatch> dexlearn(
      solver, engine, robot_model, acm2, group_robot, env, outer_batch_size);

  auto build = [&]() { dexlearn.build([&]() { env->goals(dexlearn); }); };

  if (command == "train") {

    build();

    ROS_INFO_STREAM("training");

    ros::WallTime start_time = ros::WallTime::now();

    std::ofstream logfile("log-" + envname + "-" + solvername + ".txt");

    while (true) {
      if (!ros::ok()) {
        throw std::runtime_error("aborted");
      }
      auto elapsed_time = ros::WallTime::now() - start_time;
      std::cout << "training time " << elapsed_time << " / " << training_time
                << " "
                << std::round(elapsed_time.toSec() * 100.0 /
                              training_time.toSec())
                << "% finished" << std::endl;
      if (elapsed_time > training_time) {
        std::cerr << "training finished" << std::endl;
        break;
      }
      dexlearn.step();
      dexlearn.test(true);
      std::cout << "loss " << dexlearn.solver()->loss() << std::endl;
      logfile << elapsed_time.toSec() << " " << dexlearn.solver()->loss()
              << std::endl;
      visualization_publisher.publish(dexlearn.visualization());
      robot_trajectory_publisher.publish(robot_model, group_all,
                                         dexlearn.trajectory());
    }

    logfile.close();

    if (!filename.empty()) {
      std::cerr << "saving weights to " << filename << std::endl;
      dexlearn.policyNetwork().saveWeights(filename);
      std::cerr << "weights saved" << std::endl;
    }
  }

  if (command == "test") {

    build();

    if (!filename.empty()) {
      ROS_INFO_STREAM("loading weights from " << filename);
      dexlearn.policyNetwork().loadWeights(filename);
    }

    ROS_INFO_STREAM("testing");
    while (ros::ok()) {
      dexlearn.test();
      visualization_publisher.publish(dexlearn.visualization());
      robot_trajectory_publisher.publish(robot_model, group_all,
                                         dexlearn.trajectory());
    }
  }

  if (command == "interactive") {

    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();
    robot_state.update();

    interactive_markers::InteractiveMarkerServer marker_server(
        "interactive_markers");

    InteractivePoseMarker object_marker(marker_server, "object", robot_state,
                                        0.1);

    dexlearn.setInitializer(
        [&](tractor::PhysicsSimulator<GeometryBatch> &simulator) {

          auto pose = GeometryBatch::import(
              object_marker.initialPose().inverse() * object_marker.pose());

          simulator.setBodyPose("object", pose);

        });

    build();

    if (!filename.empty()) {
      ROS_INFO_STREAM("loading weights from " << filename);
      dexlearn.policyNetwork().loadWeights(filename);
    }

    ROS_INFO_STREAM("testing");
    while (ros::ok()) {

      if (!object_marker.poll()) {
        ros::WallDuration(0.01).sleep();
        continue;
      }

      dexlearn.test();
      visualization_publisher.publish(dexlearn.visualization());
      robot_trajectory_publisher.publish(robot_model, group_all,
                                         dexlearn.trajectory());
    }
  }
}

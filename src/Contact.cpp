#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Point3D.h>
#include <mc_rtc/gui/Polygon.h>
#include <mc_rtc/gui/Polyhedron.h>
#include <mc_rtc/logging.h>

#include <ForceColl/Contact.h>

using namespace ForceColl;

FrictionPyramid::FrictionPyramid(double fricCoeff, int ridgeNum)
{
  for(int i = 0; i < ridgeNum; i++)
  {
    double theta = 2 * mc_rtc::constants::PI * (static_cast<double>(i) / ridgeNum);
    localRidgeList_.push_back(
        Eigen::Vector3d(fricCoeff * std::cos(theta), fricCoeff * std::sin(theta), 1).normalized());
  }
}

std::vector<Eigen::Vector3d> FrictionPyramid::calcGlobalRidgeList(const Eigen::Matrix3d & rot) const
{
  std::vector<Eigen::Vector3d> globalRidgeList;
  for(const auto & localRidge : localRidgeList_)
  {
    globalRidgeList.push_back(rot * localRidge);
  }
  return globalRidgeList;
}

std::shared_ptr<Contact> Contact::makeSharedFromConfig(const mc_rtc::Configuration & mcRtcConfig)
{
  if(mcRtcConfig("type") == "Empty")
  {
    return std::make_shared<EmptyContact>(mcRtcConfig);
  }
  else if(mcRtcConfig("type") == "Surface")
  {
    return std::make_shared<SurfaceContact>(mcRtcConfig);
  }
  else if(mcRtcConfig("type") == "Grasp")
  {
    return std::make_shared<GraspContact>(mcRtcConfig);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[Contact::makeSharedFromConfig] Invalid type: {}.",
                                                     mcRtcConfig("type"));
  }
}

Contact::Contact(const std::string & name, std::optional<sva::ForceVecd> maxWrench) : name_(name), maxWrench_(maxWrench)
{
}

sva::ForceVecd Contact::calcWrench(const Eigen::VectorXd & wrenchRatio, const Eigen::Vector3d & momentOrigin) const
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();
  int wrenchRatioIdx = 0;

  for(const auto & vertexWithRidge : vertexWithRidgeList_)
  {
    const Eigen::Vector3d & vertex = vertexWithRidge.vertex;
    const std::vector<Eigen::Vector3d> & ridgeList = vertexWithRidge.ridgeList;

    for(const auto & ridge : ridgeList)
    {
      Eigen::Vector3d force = wrenchRatio(wrenchRatioIdx) * ridge;
      totalWrench.force() += force;
      totalWrench.moment() += (vertex - momentOrigin).cross(force);
      wrenchRatioIdx++;
    }
  }

  assert(wrenchRatio.size() == wrenchRatioIdx);

  return totalWrench;
}

sva::ForceVecd Contact::calcLocalWrench(const Eigen::VectorXd & wrenchRatio) const
{
  assert(wrenchRatio.size() == localGraspMat_.cols());
  return {localGraspMat_ * wrenchRatio};
}

void Contact::addToGUI(mc_rtc::gui::StateBuilder & gui,
                       const std::vector<std::string> & category,
                       double forceScale,
                       double fricPyramidScale,
                       const Eigen::VectorXd & wrenchRatio)
{
  if(forceScale > 0 || fricPyramidScale > 0)
  {
    int wrenchRatioIdx = 0;
    int vertexIdx = 0;
    for(const auto & vertexWithRidge : vertexWithRidgeList_)
    {
      const Eigen::Vector3d & vertex = vertexWithRidge.vertex;
      const std::vector<Eigen::Vector3d> & ridgeList = vertexWithRidge.ridgeList;

      int ridgeIdx = 0;
      Eigen::Vector3d vertexForce = Eigen::Vector3d::Zero();
      std::vector<Eigen::Vector3d> fricPyramidVertices = {vertex};
      std::vector<std::array<size_t, 3>> fricPyramidVertexIndicies;
      for(const auto & ridge : ridgeList)
      {
        if(forceScale > 0)
        {
          Eigen::Vector3d force = wrenchRatio(wrenchRatioIdx) * ridge;
          vertexForce += force;
        }

        if(fricPyramidScale > 0)
        {
          Eigen::Vector3d fricPyramidVertex = vertex + fricPyramidScale * ridge;
          fricPyramidVertices.push_back(fricPyramidVertex);
          fricPyramidVertexIndicies.push_back(
              {0, static_cast<size_t>(ridgeIdx + 1), static_cast<size_t>(ridgeIdx + 1) % ridgeList.size() + 1});
        }

        wrenchRatioIdx++;
        ridgeIdx++;
      }

      // Add force arrow
      if(forceScale > 0)
      {
        Eigen::Vector3d arrowStart = vertex;
        Eigen::Vector3d arrowEnd = vertex + forceScale * vertexForce;
        mc_rtc::gui::ArrowConfig arrowConfig;
        arrowConfig.color = mc_rtc::gui::Color::Red;
        arrowConfig.head_diam = 0.020;
        arrowConfig.head_len = 0.03;
        arrowConfig.shaft_diam = 0.010;
        gui.addElement(category, mc_rtc::gui::Arrow(
                                     name_ + "_Force" + std::to_string(vertexIdx), arrowConfig,
                                     [arrowStart]() { return arrowStart; }, [arrowEnd]() { return arrowEnd; }));
      }

      // Add friction pyramid
      if(fricPyramidScale > 0)
      {
        mc_rtc::gui::PolyhedronConfig polyConfig;
        polyConfig.show_triangle = false;
        polyConfig.edge_config.color = mc_rtc::gui::Color(1.0, 0.6, 0.0, 1.0);
        polyConfig.show_vertices = false;
        gui.addElement(category, mc_rtc::gui::Polyhedron(
                                     name_ + "_FricPyramid" + std::to_string(vertexIdx), polyConfig,
                                     [fricPyramidVertices]() { return fricPyramidVertices; },
                                     [fricPyramidVertexIndicies]() { return fricPyramidVertexIndicies; }));
      }

      vertexIdx++;
    }
  }
}

EmptyContact::EmptyContact(const std::string & name) : Contact(name)
{
  // Set graspMat_ and vertexWithRidgeList_
  graspMat_.setZero(6, 0);
  localGraspMat_.setZero(6, 0);
}

EmptyContact::EmptyContact(const mc_rtc::Configuration & mcRtcConfig)
: EmptyContact(static_cast<std::string>(mcRtcConfig("name")))
{
}

void SurfaceContact::loadVerticesMap(const mc_rtc::Configuration & mcRtcConfig)
{
  for(const auto & verticesConfig : mcRtcConfig)
  {
    verticesMap[verticesConfig("name")] = verticesConfig("vertices");
  }
}

SurfaceContact::SurfaceContact(const std::string & name,
                               double fricCoeff,
                               const std::vector<Eigen::Vector3d> & localVertices,
                               const sva::PTransformd & pose,
                               std::optional<sva::ForceVecd> maxWrench)
: Contact(name, std::move(maxWrench))
{
  // Set graspMat_ and vertexWithRidgeList_
  FrictionPyramid fricPyramid(fricCoeff);

  graspMat_.resize(6, static_cast<Eigen::DenseIndex>(localVertices.size()) * fricPyramid.ridgeNum());
  localGraspMat_.resize(6, static_cast<Eigen::DenseIndex>(localVertices.size()) * fricPyramid.ridgeNum());

  const auto & globalRidgeList = fricPyramid.calcGlobalRidgeList(pose.rotation().transpose());

  for(size_t vertexIdx = 0; vertexIdx < localVertices.size(); vertexIdx++)
  {
    const auto & localVertex = localVertices[vertexIdx];
    Eigen::Vector3d globalVertex = (sva::PTransformd(localVertex) * pose).translation();

    for(size_t ridgeIdx = 0; ridgeIdx < globalRidgeList.size(); ridgeIdx++)
    {
      const auto & localRidge = fricPyramid.localRidgeList_[ridgeIdx];
      const auto & globalRidge = globalRidgeList[ridgeIdx];
      auto colIdx =
          static_cast<Eigen::DenseIndex>(vertexIdx) * fricPyramid.ridgeNum() + static_cast<Eigen::DenseIndex>(ridgeIdx);
      // The top 3 rows are moment, the bottom 3 rows are force.
      localGraspMat_.col(colIdx) << localVertex.cross(localRidge), localRidge;
      graspMat_.col(colIdx) << globalVertex.cross(globalRidge), globalRidge;
    }

    vertexWithRidgeList_.push_back(VertexWithRidge(globalVertex, globalRidgeList));
  }
}

SurfaceContact::SurfaceContact(const mc_rtc::Configuration & mcRtcConfig)
: SurfaceContact(mcRtcConfig("name"),
                 mcRtcConfig("fricCoeff"),
                 verticesMap.at(mcRtcConfig("verticesName")),
                 mcRtcConfig("pose"),
                 mcRtcConfig("maxWrench", std::optional<sva::ForceVecd>{}))
{
}

void SurfaceContact::addToGUI(mc_rtc::gui::StateBuilder & gui,
                              const std::vector<std::string> & category,
                              double forceScale,
                              double fricPyramidScale,
                              const Eigen::VectorXd & wrenchRatio)
{
  Contact::addToGUI(gui, category, forceScale, fricPyramidScale, wrenchRatio);

  // Add region
  {
    std::vector<Eigen::Vector3d> vertices;
    for(const auto & vertexWithRidge : vertexWithRidgeList_)
    {
      vertices.push_back(vertexWithRidge.vertex);
    }
    gui.addElement(category, mc_rtc::gui::Polygon(name_ + "_SurfaceRegion", {mc_rtc::gui::Color::Blue, 0.02},
                                                  [vertices]() { return vertices; }));
  }
}

void GraspContact::loadVerticesMap(const mc_rtc::Configuration & mcRtcConfig)
{
  for(const auto & verticesConfig : mcRtcConfig)
  {
    verticesMap[verticesConfig("name")] = verticesConfig("vertices");
  }
}

GraspContact::GraspContact(const std::string & name,
                           double fricCoeff,
                           const std::vector<sva::PTransformd> & localVertices,
                           const sva::PTransformd & pose,
                           std::optional<sva::ForceVecd> maxWrench)
: Contact(name, std::move(maxWrench))
{
  // Set graspMat_ and vertexWithRidgeList_
  FrictionPyramid fricPyramid(fricCoeff);

  graspMat_.resize(6, static_cast<Eigen::DenseIndex>(localVertices.size()) * fricPyramid.ridgeNum());
  localGraspMat_.resize(6, static_cast<Eigen::DenseIndex>(localVertices.size()) * fricPyramid.ridgeNum());

  for(size_t vertexIdx = 0; vertexIdx < localVertices.size(); vertexIdx++)
  {
    const auto & localVertexPose = localVertices[vertexIdx];
    sva::PTransformd globalVertexPose = localVertices[vertexIdx] * pose;
    const auto & localVertex = localVertexPose.translation();
    const auto & localRidgeList = fricPyramid.calcGlobalRidgeList(localVertexPose.rotation().transpose());
    const auto & globalVertex = globalVertexPose.translation();
    const auto & globalRidgeList = fricPyramid.calcGlobalRidgeList(globalVertexPose.rotation().transpose());

    for(size_t ridgeIdx = 0; ridgeIdx < globalRidgeList.size(); ridgeIdx++)
    {
      const auto & localRidge = localRidgeList[ridgeIdx];
      const auto & globalRidge = globalRidgeList[ridgeIdx];
      auto colIdx =
          static_cast<Eigen::DenseIndex>(vertexIdx) * fricPyramid.ridgeNum() + static_cast<Eigen::DenseIndex>(ridgeIdx);
      // The top 3 rows are moment, the bottom 3 rows are force.
      localGraspMat_.col(colIdx) << localVertex.cross(localRidge), localRidge;
      graspMat_.col(colIdx) << globalVertex.cross(globalRidge), globalRidge;
    }

    vertexWithRidgeList_.push_back(VertexWithRidge(globalVertex, globalRidgeList));
  }
}

GraspContact::GraspContact(const mc_rtc::Configuration & mcRtcConfig)
: GraspContact(mcRtcConfig("name"),
               mcRtcConfig("fricCoeff"),
               verticesMap.at(mcRtcConfig("verticesName")),
               mcRtcConfig("pose"),
               mcRtcConfig("maxWrench", std::optional<sva::ForceVecd>{}))
{
}

void GraspContact::addToGUI(mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category,
                            double forceScale,
                            double fricPyramidScale,
                            const Eigen::VectorXd & wrenchRatio)
{
  Contact::addToGUI(gui, category, forceScale, fricPyramidScale, wrenchRatio);

  // Add region
  {
    int vertexIdx = 0;
    for(const auto & vertexWithRidge : vertexWithRidgeList_)
    {
      Eigen::Vector3d vertex = vertexWithRidge.vertex;
      gui.addElement(category, mc_rtc::gui::Point3D(name_ + "_GraspRegion_" + std::to_string(vertexIdx),
                                                    {mc_rtc::gui::Color::Blue, 0.03}, [vertex]() { return vertex; }));
      vertexIdx++;
    }
  }
}

sva::ForceVecd ForceColl::calcTotalWrench(const std::vector<std::shared_ptr<Contact>> & contactList,
                                          const Eigen::VectorXd & wrenchRatio,
                                          const Eigen::Vector3d & momentOrigin)
{
  sva::ForceVecd totalWrench = sva::ForceVecd::Zero();
  for(const auto & wrench : calcWrenchList(contactList, wrenchRatio, momentOrigin))
  {
    totalWrench += wrench;
  }
  return totalWrench;
}

std::vector<sva::ForceVecd> ForceColl::calcWrenchList(const std::vector<std::shared_ptr<Contact>> & contactList,
                                                      const Eigen::VectorXd & wrenchRatio,
                                                      const Eigen::Vector3d & momentOrigin)
{
  std::vector<sva::ForceVecd> wrenchList;
  wrenchList.reserve(contactList.size());
  Eigen::DenseIndex wrenchRatioIdx = 0;
  for(const auto & contact : contactList)
  {
    wrenchList.push_back(contact->calcWrench(wrenchRatio.segment(wrenchRatioIdx, contact->ridgeNum()), momentOrigin));
    wrenchRatioIdx += contact->ridgeNum();
  }
  return wrenchList;
}

std::vector<sva::ForceVecd> ForceColl::calcLocalWrenchList(const std::vector<std::shared_ptr<Contact>> & contactList,
                                                           const Eigen::VectorXd & wrenchRatio)
{
  std::vector<sva::ForceVecd> wrenchList;
  wrenchList.reserve(contactList.size());
  Eigen::DenseIndex wrenchRatioIdx = 0;
  for(const auto & contact : contactList)
  {
    wrenchList.push_back(contact->calcLocalWrench(wrenchRatio.segment(wrenchRatioIdx, contact->ridgeNum())));
    wrenchRatioIdx += contact->ridgeNum();
  }
  return wrenchList;
}

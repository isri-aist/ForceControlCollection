#include <mc_rtc/constants.h>
#include <mc_rtc/gui/Arrow.h>
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

Contact::Contact(const std::string & name,
                 double fricCoeff,
                 const std::vector<Eigen::Vector3d> & localVertexList,
                 const sva::PTransformd & pose)
: name_(name)
{
  // Set graspMat_ and vertexWithRidgeList_
  FrictionPyramid fricPyramid(fricCoeff);

  graspMat_.resize(6, localVertexList.size() * fricPyramid.ridgeNum());

  const auto & globalRidgeList = fricPyramid.calcGlobalRidgeList(pose.rotation().transpose());

  for(size_t vertexIdx = 0; vertexIdx < localVertexList.size(); vertexIdx++)
  {
    Eigen::Vector3d globalVertex = (sva::PTransformd(localVertexList[vertexIdx]) * pose).translation();

    for(size_t ridgeIdx = 0; ridgeIdx < globalRidgeList.size(); ridgeIdx++)
    {
      const auto & globalRidge = globalRidgeList[ridgeIdx];
      // The top 3 rows are moment, the bottom 3 rows are force.
      graspMat_.col(vertexIdx * fricPyramid.ridgeNum() + ridgeIdx) << globalVertex.cross(globalRidge), globalRidge;
    }

    vertexWithRidgeList_.push_back(VertexWithRidge(globalVertex, globalRidgeList));
  }
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

void Contact::addToGUI(mc_rtc::gui::StateBuilder & gui,
                       const std::vector<std::string> & category,
                       const Eigen::VectorXd & wrenchRatio,
                       double forceScale,
                       double fricPyramidScale)
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
      Eigen::Vector3d force = wrenchRatio(wrenchRatioIdx) * ridge;
      vertexForce += force;
      Eigen::Vector3d fricPyramidVertex = vertex + fricPyramidScale * ridge;
      fricPyramidVertices.push_back(fricPyramidVertex);
      fricPyramidVertexIndicies.push_back(
          {0, static_cast<size_t>(ridgeIdx + 1), static_cast<size_t>(ridgeIdx + 1) % ridgeList.size() + 1});

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
      // \todo When https://github.com/jrl-umi3218/mc_rtc/issues/334 is resolved, replace with the following lines
      // gui.addElement(category, mc_rtc::gui::Polyhedron(
      //                              name_ + "_FricPyramid" + std::to_string(vertexIdx), polyConfig,
      //                              [fricPyramidVertices]() { return fricPyramidVertices; },
      //                              [fricPyramidVertexIndicies]() { return fricPyramidVertexIndicies; }));
      gui.addElement(category, mc_rtc::gui::Polyhedron(
                                   name_ + "_FricPyramid" + std::to_string(vertexIdx), polyConfig,
                                   [fricPyramidVertices]() { return fricPyramidVertices; },
                                   [fricPyramidVertexIndicies]() { return fricPyramidVertexIndicies; },
                                   [fricPyramidVertices]() {
                                     return std::vector<mc_rtc::gui::Color>(fricPyramidVertices.size(),
                                                                            mc_rtc::gui::Color(1.0, 0.6, 0.0, 1.0));
                                   }));
    }

    vertexIdx++;
  }
}

/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <ForceColl/Contact.h>

TEST(TestContact, SurfaceContact)
{
  const std::string vertexListYamlStr = R"(
- name: vertexListSample
  vertices:
    - [0, 0, 0]
    - [0.1, -0.2, 0.3]
)";
  const std::string constructorYamlStr = R"(
type: Surface
name: ContactYaml
fricCoeff: 1.0
vertexListName: vertexListSample
pose:
  translation: [0.0, 0.5, -0.5]
  rotation: [1.5707963267948966, 0, 0]
)";

  auto contactDirect = std::make_shared<ForceColl::SurfaceContact>(
      "ContactDirect", 1.0, std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, -0.2, 0.3)},
      sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5)));

  ForceColl::SurfaceContact::loadVertexListMap(mc_rtc::Configuration::fromYAMLData(vertexListYamlStr));
  auto contactYaml = ForceColl::Contact::makeSharedFromConfig(mc_rtc::Configuration::fromYAMLData(constructorYamlStr));

  EXPECT_LT((contactDirect->graspMat_ - contactYaml->graspMat_).norm(), 1e-8) << "contactDirect:\n"
                                                                              << contactDirect->graspMat_ << std::endl
                                                                              << "contactYaml:\n"
                                                                              << contactYaml->graspMat_ << std::endl;
}

TEST(TestContact, GraspContact)
{
  const std::string vertexListYamlStr = R"(
- name: vertexListSample
  vertices:
    - translation: [0.0, -0.05, 0.0]
      rotation: [-1.5707963267948966, 0, 0]
    - translation: [0.0, 0.05, 0.0]
      rotation: [1.5707963267948966, 0, 0]
)";
  const std::string constructorYamlStr = R"(
type: Grasp
name: ContactYaml
fricCoeff: 1.0
vertexListName: vertexListSample
pose:
  translation: [0.0, 0.5, -0.5]
  rotation: [0, 1.5707963267948966, 0]
)";

  auto contactDirect = std::make_shared<ForceColl::GraspContact>(
      "ContactDirect", 1.0,
      std::vector<sva::PTransformd>{sva::PTransformd(sva::RotX(-1 * M_PI / 2), Eigen::Vector3d(0.0, -0.05, 0.0)),
                                    sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.05, 0.0))},
      sva::PTransformd(sva::RotY(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5)));

  ForceColl::GraspContact::loadVertexListMap(mc_rtc::Configuration::fromYAMLData(vertexListYamlStr));
  auto contactYaml = ForceColl::Contact::makeSharedFromConfig(mc_rtc::Configuration::fromYAMLData(constructorYamlStr));

  EXPECT_LT((contactDirect->graspMat_ - contactYaml->graspMat_).norm(), 1e-6) << "contactDirect:\n"
                                                                              << contactDirect->graspMat_ << std::endl
                                                                              << "contactYaml:\n"
                                                                              << contactYaml->graspMat_ << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

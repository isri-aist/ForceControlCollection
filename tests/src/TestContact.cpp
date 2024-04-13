#include <gtest/gtest.h>

#include <ForceColl/Contact.h>

TEST(TestContact, EmptyContact)
{
  const std::string constructorYamlStr = R"(
type: Empty
name: ContactYaml
)";

  auto contactDirect = std::make_shared<ForceColl::EmptyContact>(std::string("ContactDirect"));

  auto contactYaml = ForceColl::Contact::makeSharedFromConfig(mc_rtc::Configuration::fromYAMLData(constructorYamlStr));

  EXPECT_LT((contactDirect->graspMat_ - contactYaml->graspMat_).norm(), 1e-8) << "contactDirect:\n"
                                                                              << contactDirect->graspMat_ << std::endl
                                                                              << "contactYaml:\n"
                                                                              << contactYaml->graspMat_ << std::endl;
  EXPECT_TRUE(contactDirect->maxWrench_ == contactYaml->maxWrench_);
}

TEST(TestContact, SurfaceContact)
{
  const std::string verticesYamlStr = R"(
- name: verticesSample
  vertices:
    - [0, 0, 0]
    - [0.1, -0.2, 0.3]
)";
  const std::string constructorYamlStr = R"(
type: Surface
name: ContactYaml
fricCoeff: 1.0
verticesName: verticesSample
pose:
  translation: [0.0, 0.5, -0.5]
  rotation: [1.5707963267948966, 0, 0]
maxWrench:
  force: [10.0, 10.0, 100.0]
  couple: [2.0, 2.0, 2.0]
)";

  auto contactDirect = std::make_shared<ForceColl::SurfaceContact>(
      "ContactDirect", 1.0, std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.1, -0.2, 0.3)},
      sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5)),
      sva::ForceVecd(Eigen::Vector3d(2.0, 2.0, 2.0), Eigen::Vector3d(10.0, 10.0, 100.0)));

  ForceColl::SurfaceContact::loadVerticesMap(mc_rtc::Configuration::fromYAMLData(verticesYamlStr));
  auto contactYaml = ForceColl::Contact::makeSharedFromConfig(mc_rtc::Configuration::fromYAMLData(constructorYamlStr));

  EXPECT_LT((contactDirect->graspMat_ - contactYaml->graspMat_).norm(), 1e-8) << "contactDirect:\n"
                                                                              << contactDirect->graspMat_ << std::endl
                                                                              << "contactYaml:\n"
                                                                              << contactYaml->graspMat_ << std::endl;
  EXPECT_TRUE(contactDirect->maxWrench_ == contactYaml->maxWrench_);
}

TEST(TestContact, GraspContact)
{
  const std::string verticesYamlStr = R"(
- name: verticesSample
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
verticesName: verticesSample
pose:
  translation: [0.0, 0.5, -0.5]
  rotation: [0, 1.5707963267948966, 0]
maxWrench:
  force: [20.0, 20.0, 100.0]
  couple: [1.0, 1.0, 1.0]
)";

  auto contactDirect = std::make_shared<ForceColl::GraspContact>(
      "ContactDirect", 1.0,
      std::vector<sva::PTransformd>{sva::PTransformd(sva::RotX(-1 * M_PI / 2), Eigen::Vector3d(0.0, -0.05, 0.0)),
                                    sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.05, 0.0))},
      sva::PTransformd(sva::RotY(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5)),
      sva::ForceVecd(Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(20.0, 20.0, 100.0)));

  ForceColl::GraspContact::loadVerticesMap(mc_rtc::Configuration::fromYAMLData(verticesYamlStr));
  auto contactYaml = ForceColl::Contact::makeSharedFromConfig(mc_rtc::Configuration::fromYAMLData(constructorYamlStr));

  EXPECT_LT((contactDirect->graspMat_ - contactYaml->graspMat_).norm(), 1e-6) << "contactDirect:\n"
                                                                              << contactDirect->graspMat_ << std::endl
                                                                              << "contactYaml:\n"
                                                                              << contactYaml->graspMat_ << std::endl;
  EXPECT_TRUE(contactDirect->maxWrench_ == contactYaml->maxWrench_);
}

TEST(TestContact, calcWrench)
{
  using Limb = std::string;

  double fricCoeff = 0.5;
  auto leftFootContact = std::make_shared<ForceColl::SurfaceContact>(
      "LeftFootContact", fricCoeff,
      std::vector<Eigen::Vector3d>{Eigen::Vector3d(-0.1, -0.1, 0.0), Eigen::Vector3d(-0.1, 0.1, 0.0),
                                   Eigen::Vector3d(0.1, 0.0, 0.0)},
      sva::PTransformd::Identity());
  auto rightFootContact = std::make_shared<ForceColl::SurfaceContact>(
      "RightFootContact", fricCoeff, std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()},
      sva::PTransformd(Eigen::Vector3d(0, -0.5, 0.5)));
  auto leftHandContact = std::make_shared<ForceColl::EmptyContact>(std::string("LefttHandContact"));

  std::vector<Limb> limbList = {"LeftFoot", "RightFoot", "LeftHand"};
  std::vector<std::shared_ptr<ForceColl::Contact>> contactList = {leftFootContact, rightFootContact, leftHandContact};
  std::map<Limb, std::shared_ptr<ForceColl::Contact>> contactMap;
  std::unordered_map<Limb, std::shared_ptr<ForceColl::Contact>> contactUnorderedMap;
  for(size_t i = 0; i < contactList.size(); i++)
  {
    contactMap.emplace(limbList[i], contactList[i]);
    contactUnorderedMap.emplace(limbList[i], contactList[i]);
  }

  int wrenchRatioIdx = 0;
  for(const auto & contact : contactList)
  {
    wrenchRatioIdx += contact->ridgeNum();
  }
  Eigen::VectorXd wrenchRatio = Eigen::VectorXd::Random(wrenchRatioIdx);

  ForceColl::calcTotalWrench(contactList, wrenchRatio);
  ForceColl::calcTotalWrench(ForceColl::getContactVecFromMap(contactMap), wrenchRatio);
  ForceColl::calcTotalWrench(ForceColl::getContactVecFromMap(contactUnorderedMap), wrenchRatio);

  ForceColl::calcWrenchList(contactList, wrenchRatio);
  ForceColl::calcWrenchList(contactMap, wrenchRatio);
  ForceColl::calcWrenchList(contactUnorderedMap, wrenchRatio);
  ForceColl::calcWrenchList(ForceColl::getContactVecFromMap(contactMap), wrenchRatio);
  ForceColl::calcWrenchList(ForceColl::getContactVecFromMap(contactUnorderedMap), wrenchRatio);
}

TEST(TestContact, UpdateSurfaceContactVertices)
{
  sva::PTransformd targetPose(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5));
  std::vector<Eigen::Vector3d> localVertices = {Eigen::Vector3d(-0.1, -0.1, 0.0), Eigen::Vector3d(-0.1, 0.1, 0.0),
                                                Eigen::Vector3d(0.0, 0.0, 0.1)};
  auto originalContact =
      std::make_shared<ForceColl::SurfaceContact>("OriginalContact", 1.0, localVertices, sva::PTransformd::Identity());
  auto targetContact = std::make_shared<ForceColl::SurfaceContact>("TargetContact", 1.0, localVertices, targetPose);
  originalContact->updateGlobalVertices(targetPose);
  EXPECT_LT((originalContact->graspMat_ - targetContact->graspMat_).norm(), 1e-8)
      << "originalContact:\n"
      << originalContact->graspMat_ << std::endl
      << "targetContact:\n"
      << targetContact->graspMat_ << std::endl;
}

TEST(TestContact, UpdateGraspContactVertices)
{
  sva::PTransformd targetPose(sva::RotX(M_PI / 2), Eigen::Vector3d(0.0, 0.5, -0.5));
  std::vector<sva::PTransformd> localVertices = {
      sva::PTransformd(sva::RotX(-1 * M_PI / 2), Eigen::Vector3d(-0.1, -0.1, 0.0)),
      sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(-0.1, 0.1, 0.0)),
      sva::PTransformd(sva::RotX(0.0), Eigen::Vector3d(0.0, 0.0, 0.1))};
  auto originalContact =
      std::make_shared<ForceColl::GraspContact>("OriginalContact", 1.0, localVertices, sva::PTransformd::Identity());
  auto targetContact = std::make_shared<ForceColl::GraspContact>("TargetContact", 1.0, localVertices, targetPose);
  originalContact->updateGlobalVertices(targetPose);
  EXPECT_LT((originalContact->graspMat_ - targetContact->graspMat_).norm(), 1e-8)
      << "originalContact:\n"
      << originalContact->graspMat_ << std::endl
      << "targetContact:\n"
      << targetContact->graspMat_ << std::endl;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <ForceColl/WrenchDistribution.h>

TEST(TestWrenchDistribution, TwoSurfaceContact)
{
  enum class Foot
  {
    Left,
    Right
  };

  double fricCoeff = 0.5;
  auto leftFootContact = std::make_shared<ForceColl::SurfaceContact>(
      "LeftFootContact", fricCoeff,
      std::vector<Eigen::Vector3d>{Eigen::Vector3d(-0.1, -0.1, 0.0), Eigen::Vector3d(-0.1, 0.1, 0.0),
                                   Eigen::Vector3d(0.1, 0.0, 0.0)},
      sva::PTransformd::Identity());
  auto rightFootContact = std::make_shared<ForceColl::SurfaceContact>(
      "RightFootContact", fricCoeff, std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()},
      sva::PTransformd(Eigen::Vector3d(0, -0.5, 0.5)));
  std::unordered_map<Foot, std::shared_ptr<ForceColl::Contact>> contactList = {{Foot::Left, leftFootContact},
                                                                               {Foot::Right, rightFootContact}};

  sva::ForceVecd desiredTotalWrench = sva::ForceVecd(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 500.0));
  auto wrenchDist = std::make_shared<ForceColl::WrenchDistribution<Foot>>(contactList);
  sva::ForceVecd resultTotalWrench = wrenchDist->run(desiredTotalWrench);

  EXPECT_LT((desiredTotalWrench.vector() - resultTotalWrench.vector()).norm(), 1e-2)
      << "desiredTotalWrench: " << desiredTotalWrench << std::endl
      << "resultTotalWrench: " << resultTotalWrench << std::endl;
  EXPECT_TRUE((wrenchDist->resultWrenchRatio_.array() > 0).all());
  for(const auto & wrenchKV : wrenchDist->calcWrenchList())
  {
    EXPECT_GT(wrenchKV.second.vector().norm(), 1e-10);
  }
}

TEST(TestWrenchDistribution, ContainsEmptyContact)
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
  std::unordered_map<Limb, std::shared_ptr<ForceColl::Contact>> contactList = {
      {"LeftFoot", leftFootContact}, {"RightFoot", rightFootContact}, {"LeftHand", leftHandContact}};

  sva::ForceVecd desiredTotalWrench = sva::ForceVecd(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 500.0));
  auto wrenchDist = std::make_shared<ForceColl::WrenchDistribution<Limb>>(contactList);
  sva::ForceVecd resultTotalWrench = wrenchDist->run(desiredTotalWrench);

  EXPECT_LT((desiredTotalWrench.vector() - resultTotalWrench.vector()).norm(), 1e-2)
      << "desiredTotalWrench: " << desiredTotalWrench << std::endl
      << "resultTotalWrench: " << resultTotalWrench << std::endl;
  EXPECT_TRUE((wrenchDist->resultWrenchRatio_.array() > 0).all());
  for(const auto & wrenchKV : wrenchDist->calcWrenchList())
  {
    if(wrenchKV.first == "LeftHand")
    {
      EXPECT_LT(wrenchKV.second.vector().norm(), 1e-10);
    }
    else
    {
      EXPECT_GT(wrenchKV.second.vector().norm(), 1e-10);
    }
  }
}

TEST(TestWrenchDistribution, ContainsGraspContact)
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
  auto leftHandContact = std::make_shared<ForceColl::GraspContact>(
      "LeftHandContact", fricCoeff,
      std::vector<sva::PTransformd>{sva::PTransformd::Identity(), sva::PTransformd(sva::RotX(M_PI))},
      sva::PTransformd(sva::RotY(M_PI / 2), Eigen::Vector3d(0.5, 0.5, 1.0)));
  std::unordered_map<Limb, std::shared_ptr<ForceColl::Contact>> contactList = {
      {"LeftFoot", leftFootContact}, {"RightFoot", rightFootContact}, {"LeftHand", leftHandContact}};

  sva::ForceVecd desiredTotalWrench = sva::ForceVecd(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 500.0));
  auto wrenchDist = std::make_shared<ForceColl::WrenchDistribution<Limb>>(contactList);
  sva::ForceVecd resultTotalWrench = wrenchDist->run(desiredTotalWrench);

  EXPECT_LT((desiredTotalWrench.vector() - resultTotalWrench.vector()).norm(), 1e-2)
      << "desiredTotalWrench: " << desiredTotalWrench << std::endl
      << "resultTotalWrench: " << resultTotalWrench << std::endl;
  EXPECT_TRUE((wrenchDist->resultWrenchRatio_.array() > 0).all());
  for(const auto & wrenchKV : wrenchDist->calcWrenchList())
  {
    EXPECT_GT(wrenchKV.second.vector().norm(), 1e-10);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

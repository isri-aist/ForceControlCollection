/* Author: Masaki Murooka */

#include <gtest/gtest.h>

#include <ForceColl/WrenchDistribution.h>

enum class Foot
{
  Left,
  Right
};

TEST(TestWrenchDistribution, Test1)
{
  double fricCoeff = 0.5;
  auto leftContact = std::make_shared<ForceColl::Contact>("LeftContact", fricCoeff,
                                                          std::vector<Eigen::Vector3d>{Eigen::Vector3d(-0.1, -0.1, 0.0),
                                                                                       Eigen::Vector3d(-0.1, 0.1, 0.0),
                                                                                       Eigen::Vector3d(0.1, 0.0, 0.0)},
                                                          sva::PTransformd::Identity());
  auto rightContact = std::make_shared<ForceColl::Contact>(
      "RightContact", fricCoeff, std::vector<Eigen::Vector3d>{Eigen::Vector3d::Zero()},
      sva::PTransformd(sva::RotX(M_PI / 2), Eigen::Vector3d(0, 0.5, 0.5)));
  std::unordered_map<Foot, std::shared_ptr<ForceColl::Contact>> contactList = {{Foot::Left, leftContact},
                                                                               {Foot::Right, rightContact}};

  sva::ForceVecd desiredTotalWrench =
      sva::ForceVecd(Eigen::Vector3d(100.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 500.0));
  auto wrenchDist = std::make_shared<ForceColl::WrenchDistribution<Foot>>(contactList);
  sva::ForceVecd resultTotalWrench = wrenchDist->run(desiredTotalWrench);

  EXPECT_LT((desiredTotalWrench.vector() - resultTotalWrench.vector()).norm(), 1e-2)
      << "desiredTotalWrench: " << desiredTotalWrench << std::endl
      << "resultTotalWrench: " << resultTotalWrench << std::endl;
  EXPECT_TRUE((wrenchDist->resultWrenchRatio_.array() > 0).all());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

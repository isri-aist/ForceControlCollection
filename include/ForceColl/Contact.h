#pragma once

#include <unordered_map>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <ForceColl/Constants.h>

namespace ForceColl
{
/** \brief Friction pyramid. */
class FrictionPyramid
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param fricCoeff friction coefficient
      \param ridgeNum number of ridges of friction pyramid
   */
  FrictionPyramid(double fricCoeff, int ridgeNum = 4);

  /** \brief Calculate ridge vector list in global coordinates. */
  std::vector<Eigen::Vector3d> calcGlobalRidgeList(const Eigen::Matrix3d & rot) const;

  /** \brief Number of ridges. */
  inline int ridgeNum() const
  {
    return static_cast<int>(localRidgeList_.size());
  }

public:
  //! Local ridge list
  std::vector<Eigen::Vector3d> localRidgeList_;
};

/** \brief Contact. */
class Contact
{
public:
  /** \brief Vertex with ridges. */
  struct VertexWithRidge
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Vertex
    Eigen::Vector3d vertex;

    //! Global ridge list
    std::vector<Eigen::Vector3d> ridgeList;

    /** \brief Constructor.
        \param _vertex vertex
        \param _ridgeList global ridge list
     */
    VertexWithRidge(const Eigen::Vector3d & _vertex, const std::vector<Eigen::Vector3d> & _ridgeList)
    : vertex(_vertex), ridgeList(_ridgeList)
    {
    }
  };

public:
  /** \brief Make shared pointer from mc_rtc configuration.
      \param mcRtcConfig mc_rtc configuration
  */
  static std::shared_ptr<Contact> makeSharedFromConfig(const mc_rtc::Configuration & mcRtcConfig);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param name name of contact
   */
  Contact(const std::string & name);

  /** \brief Get type of contact. */
  virtual std::string type() const = 0;

  /** \brief Calculate wrench.
      \param wrenchRatio wrench ratio of each ridge
      \param momentOrigin moment origin
      \returns contact wrench
   */
  sva::ForceVecd calcWrench(const Eigen::VectorXd & wrenchRatio,
                            const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero()) const;

  /** \brief Add markers to GUI.
      \param gui GUI
      \param category category of GUI entries
      \param forceScale scale of force markers (set non-positive for no visualization)
      \param fricPyramidScale scale of friction pyramid markers (set non-positive for no visualization)
      \param wrenchRatio wrench ratio of each ridge
   */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui,
                        const std::vector<std::string> & category,
                        double forceScale = constants::defaultForceScale,
                        double fricPyramidScale = constants::defaultFricPyramidScale,
                        const Eigen::VectorXd & wrenchRatio = Eigen::VectorXd::Zero(0));

public:
  //! Name of contact
  std::string name_;

  //! Grasp matrix
  Eigen::Matrix<double, 6, Eigen::Dynamic> graspMat_;

  //! List of vertex with ridges
  std::vector<VertexWithRidge> vertexWithRidgeList_;
};

/** \brief Empty contact. */
class EmptyContact : public Contact
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param name name of contact
   */
  EmptyContact(const std::string & name);

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
  */
  EmptyContact(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Get type of contact. */
  inline virtual std::string type() const override
  {
    return "Empty";
  }
};

/** \brief Surface contact. */
class SurfaceContact : public Contact
{
public:
  /** \brief Load map of surface vertices in local coordinates
      \param mcRtcConfig mc_rtc configuration
  */
  static void loadVerticesMap(const mc_rtc::Configuration & mcRtcConfig);

  //! Map of surface vertices in local coordinates
  static inline std::unordered_map<std::string, std::vector<Eigen::Vector3d>> verticesMap;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param name name of contact
      \param fricCoeff friction coefficient
      \param localVertices surface vertices in local coordinates
      \param pose pose of contact
   */
  SurfaceContact(const std::string & name,
                 double fricCoeff,
                 const std::vector<Eigen::Vector3d> & localVertices,
                 const sva::PTransformd & pose);

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
  */
  SurfaceContact(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Get type of contact. */
  inline virtual std::string type() const override
  {
    return "Surface";
  }

  /** \brief Add markers to GUI.
      \param gui GUI
      \param category category of GUI entries
      \param forceScale scale of force markers (set non-positive for no visualization)
      \param fricPyramidScale scale of friction pyramid markers (set non-positive for no visualization)
      \param wrenchRatio wrench ratio of each ridge
   */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui,
                        const std::vector<std::string> & category,
                        double forceScale = constants::defaultForceScale,
                        double fricPyramidScale = constants::defaultFricPyramidScale,
                        const Eigen::VectorXd & wrenchRatio = Eigen::VectorXd::Zero(0)) override;
};

/** \brief Grasp contact. */
class GraspContact : public Contact
{
public:
  /** \brief Load map of grasp vertices in local coordinates
      \param mcRtcConfig mc_rtc configuration
  */
  static void loadVerticesMap(const mc_rtc::Configuration & mcRtcConfig);

  //! Map of grasp vertices in local coordinates
  static inline std::unordered_map<std::string, std::vector<sva::PTransformd>> verticesMap;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor.
      \param name name of contact
      \param fricCoeff friction coefficient
      \param localVertices grasp vertices in local coordinates
      \param pose pose of contact
   */
  GraspContact(const std::string & name,
               double fricCoeff,
               const std::vector<sva::PTransformd> & localVertices,
               const sva::PTransformd & pose);

  /** \brief Constructor.
      \param mcRtcConfig mc_rtc configuration
  */
  GraspContact(const mc_rtc::Configuration & mcRtcConfig);

  /** \brief Get type of contact. */
  inline virtual std::string type() const override
  {
    return "Grasp";
  }

  /** \brief Add markers to GUI.
      \param gui GUI
      \param category category of GUI entries
      \param forceScale scale of force markers (set non-positive for no visualization)
      \param fricPyramidScale scale of friction pyramid markers (set non-positive for no visualization)
      \param wrenchRatio wrench ratio of each ridge
   */
  virtual void addToGUI(mc_rtc::gui::StateBuilder & gui,
                        const std::vector<std::string> & category,
                        double forceScale = constants::defaultForceScale,
                        double fricPyramidScale = constants::defaultFricPyramidScale,
                        const Eigen::VectorXd & wrenchRatio = Eigen::VectorXd::Zero(0)) override;
};

/** \brief Calculate total wrench.
    \param contactList list of contact constraint
    \param wrenchRatio wrench ratio
    \param momentOrigin moment origin
    \returns total wrench
*/
sva::ForceVecd calcTotalWrench(const std::vector<std::shared_ptr<Contact>> & contactList,
                               const Eigen::VectorXd & wrenchRatio,
                               const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero());

/** \brief Calculate contact wrench list.
    \param contactList list of contact constraint
    \param wrenchRatio wrench ratio
    \param momentOrigin moment origin
    \returns contact wrench list
*/
std::vector<sva::ForceVecd> calcWrenchList(const std::vector<std::shared_ptr<Contact>> & contactList,
                                           const Eigen::VectorXd & wrenchRatio,
                                           const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero());

/** \brief Calculate contact wrench list.
    \tparam MapType type of map container
    \tparam KeyType key type
    \param contactList list of contact constraint
    \param wrenchRatio wrench ratio
    \param momentOrigin moment origin
    \returns contact wrench list
*/
template<template<class...> class MapType, class KeyType, class... RestTypes>
MapType<KeyType, sva::ForceVecd> calcWrenchList(
    const MapType<KeyType, std::shared_ptr<Contact>, RestTypes...> & contactList,
    const Eigen::VectorXd & wrenchRatio,
    const Eigen::Vector3d & momentOrigin = Eigen::Vector3d::Zero());

/** \brief Convert vector of contact constraint to map.
    \tparam MapType type of map container
    \tparam KeyType key type
    \param contactList vector of contact constraint
    \returns map of contact constraint
*/
template<template<class...> class MapType, class KeyType, class... RestTypes>
std::vector<std::shared_ptr<Contact>> getContactVecFromMap(
    const MapType<KeyType, std::shared_ptr<Contact>, RestTypes...> & contactList);
} // namespace ForceColl

#include <ForceColl/Contact.hpp>

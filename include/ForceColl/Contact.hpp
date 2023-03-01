namespace ForceColl
{
template<template<class, class> class MapType, class KeyType>
MapType<KeyType, sva::ForceVecd> calcWrenchList(const MapType<KeyType, std::shared_ptr<Contact>> & contactList,
                                                const Eigen::VectorXd & wrenchRatio,
                                                const Eigen::Vector3d & momentOrigin)
{
  MapType<KeyType, sva::ForceVecd> wrenchList;
  int wrenchRatioIdx = 0;
  for(const auto & contactKV : contactList)
  {
    wrenchList.emplace(contactKV.first,
                       contactKV.second->calcWrench(
                           wrenchRatio.segment(wrenchRatioIdx, contactKV.second->graspMat_.cols()), momentOrigin));
    wrenchRatioIdx += static_cast<int>(contactKV.second->graspMat_.cols());
  }
  return wrenchList;
}

template<template<class, class> class MapType, class KeyType>
std::vector<std::shared_ptr<Contact>> getContactVecFromMap(
    const MapType<KeyType, std::shared_ptr<Contact>> & contactList)
{
  std::vector<std::shared_ptr<Contact>> contactVec;
  for(const auto & contactKV : contactList)
  {
    contactVec.push_back(contactKV.second);
  }
  return contactVec;
}
} // namespace ForceColl
